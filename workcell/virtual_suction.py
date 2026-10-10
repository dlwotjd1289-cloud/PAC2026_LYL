"""Gazebo 전용 가상 접촉. 기본 실행에는 설치되지 않는 선택 기능."""
from __future__ import annotations

import math
import subprocess


def require_gazebo():
    """실행 중인 기존 작업셀을 확인하고, 확인 실패 시 동작 전에 중단한다."""
    result = subprocess.run(['ign', 'topic', '-l'], capture_output=True, text=True,
                            timeout=5, check=True)
    prefix = '/world/ahead_workcell_v4_2_physical_scale/'
    if not any(line.strip().startswith(prefix) for line in result.stdout.splitlines()):
        raise RuntimeError('가상 흡착: 실행 중인 Gazebo 작업셀을 확인하지 못했습니다.')


def install(module):
    """원본 grip을 재사용하며 압착만 가상 접촉으로 바꾼다.

    GRIPPED를 직접 대입하지 않는다. 기존 흡착 노드와 Gazebo DetachableJoint가
    부착을 처리하며, 원본 상승·추종·경로·적재 검증은 그대로 실행된다.
    """
    cls = module.MoveItPickPlace
    original_grip = cls.grip
    original_vacuum = cls.set_vacuum
    place_gap = module.PLACE_GAP_M

    class ContactProxy:
        def __init__(self, node, guard, top, size):
            self.node, self.guard, self.top, self.size = node, guard, top, size
            self.checked = False

        def __getattr__(self, name):
            return getattr(self.guard, name)

        def descend(self, z, yaw, what, avoid_collisions=True):
            if what != 'press cup on box':
                result = self.guard.descend(z, yaw, what, avoid_collisions=avoid_collisions)
                if what == 'descend to box (collision-checked)' and avoid_collisions:
                    self.checked = True
                return result
            if not self.checked:
                raise module.Failure('가상 흡착: 충돌 검사 하강이 완료되지 않았습니다.')
            tcp_z = self.node.tcp_z()
            if tcp_z is None or not math.isfinite(tcp_z):
                raise module.Failure('가상 흡착: TCP 높이를 확인하지 못했습니다.')
            if abs(tcp_z - (self.top + module.CHECKED_GAP_M)) > 0.002:
                raise module.Failure('가상 흡착: 접촉 대기 높이 오차가 2 mm를 넘었습니다.')
            box_top = float(self.node.fresh_box()[0][2]) + float(self.size[2]) / 2
            gap = tcp_z - box_top
            if not math.isfinite(gap) or not 0 <= gap <= 0.015:
                raise module.Failure('가상 흡착: 박스 상면과 TCP 간격이 0~15 mm 밖입니다.')
            # 부착 순간의 상대 간격을 보존하는 조인트이므로, 해제 높이에도 반영한다.
            # 이를 빼면 목적지에서 박스가 표면 아래로 내려갈 수 있다.
            module.PLACE_GAP_M = place_gap + gap
            self.node._virtual_contact_ready = True
            print(f'>>> [흡착] 가상 접촉 사용; 물리 압착 생략; 간격 {gap*1000:.2f} mm', flush=True)

    def grip(self, b, yaw, top, size, q, at_pick):
        if not self.get_parameter('use_sim_time').value:
            raise module.Failure('가상 흡착은 시뮬레이션 시간 사용 시에만 가능합니다.')
        self._virtual_contact_ready = False
        if hasattr(self, 'vertical'):
            # Compatibility with an executor that exposes a vertical descent guard.
            original_guard = self.vertical
            self.vertical = ContactProxy(self, original_guard, top, size)
            try:
                return original_grip(self, b, yaw, top, size, q, at_pick)
            finally:
                self.vertical = original_guard
                self._virtual_contact_ready = False

        # The original V4.6 MoveIt executor descends through self.line(), not
        # self.vertical. Intercept ONLY the final, unchecked cup press.
        original_line = self.line
        contact = ContactProxy(self, None, top, size)

        def guarded_line(xyz, target_yaw, what, *args, **kwargs):
            if what == 'press cup on box':
                return contact.descend(xyz[2], target_yaw, what,
                                       avoid_collisions=kwargs.get('avoid_collisions', True))
            result = original_line(xyz, target_yaw, what, *args, **kwargs)
            if what == 'descend to box (collision-checked)' and kwargs.get('avoid_collisions', True):
                contact.checked = True
            return result

        self.line = guarded_line
        try:
            return original_grip(self, b, yaw, top, size, q, at_pick)
        finally:
            self.line = original_line
            self._virtual_contact_ready = False

    def set_vacuum(self, on):
        if not on:
            return original_vacuum(self, on)
        if not getattr(self, '_virtual_contact_ready', False):
            raise module.Failure('가상 흡착: 확인된 접근 없이 부착할 수 없습니다.')
        if not hasattr(self, '_virtual_touch_pub'):
            self._virtual_touch_pub = self.create_publisher(module.Bool, '/pac/suction/touched', 10)
        pub = self._virtual_touch_pub
        self.wait(lambda: pub.get_subscription_count() > 0, 3, 'virtual contact subscriber')
        original_vacuum(self, on)
        self.wait(lambda: self.suction in ('SEARCHING', 'GRIPPED'), 3, 'vacuum state')
        # 기존 노드가 이 접촉 이벤트를 받아 Gazebo attach를 발행한다.
        # 성공 상태를 조작하지 않으며 원본 grip의 GRIPPED 대기와 상승 확인을 따른다.
        for _ in range(3):
            if self.suction == 'GRIPPED':
                break
            pub.publish(module.Bool(data=True))
            self.spin_for(0.1)

    cls.grip = grip
    cls.set_vacuum = set_vacuum
