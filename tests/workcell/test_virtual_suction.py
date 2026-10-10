"""가상 접촉 분기 회귀. 실제 ROS/Gazebo 실행을 대신하지 않는다."""
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from workcell.virtual_suction import install, require_gazebo


def fixture_module():
    class Failure(RuntimeError):
        pass

    class Guard:
        def __init__(self):
            self.steps = []
        def descend(self, z, yaw, what, avoid_collisions=True):
            self.steps.append((what, avoid_collisions))

    class Node:
        def __init__(self):
            self.vertical = Guard()
            self.guard = self.vertical
            self.suction = 'OFF'
            self.z = 1.01
            self.sim_time = True
            self.attached = False
            self.events = []
            self.fail_lift = False
        def get_parameter(self, name):
            assert name == 'use_sim_time'
            return SimpleNamespace(value=self.sim_time)
        def tcp_z(self):
            return self.z
        def fresh_box(self):
            return ([0, 0, .5], None, 0)
        def create_publisher(self, typ, topic, depth):
            assert topic == '/pac/suction/touched'
            def publish(msg):
                assert self.suction == 'SEARCHING'
                self.events.append('contact')
                self.attached = True
                self.suction = 'GRIPPED'
            return SimpleNamespace(get_subscription_count=lambda: 1, publish=publish)
        def wait(self, pred, timeout, label):
            if not pred():
                raise Failure(label)
        def spin_for(self, duration):
            pass
        def set_vacuum(self, on):
            self.events.append(('vacuum', on))
            self.suction = 'SEARCHING' if on else 'OFF'
        def grip(self, b, yaw, top, size, q, at_pick):
            self.vertical.descend(1.01, 0, 'descend to box (collision-checked)')
            self.vertical.descend(.999, 0, 'press cup on box', avoid_collisions=False)
            self.set_vacuum(True)
            self.wait(lambda: self.suction == 'GRIPPED', 5, 'GRIPPED')
            self.events.append('lift-and-follow-check')
            if self.fail_lift:
                raise Failure('box did not follow')
            return .14
    return SimpleNamespace(MoveItPickPlace=Node, Bool=SimpleNamespace, Failure=Failure,
                           PLACE_GAP_M=.005, CHECKED_GAP_M=.010)


def run_grip(node):
    return node.grip([0, 0, .5], 0, 1., [1, 1, 1], None, True)


def test_only_press_is_replaced_and_release_gap_is_compensated():
    mod = fixture_module()
    install(mod)
    node = mod.MoveItPickPlace()
    assert run_grip(node) == .14
    assert node.guard.steps == [('descend to box (collision-checked)', True)]
    assert node.events == [('vacuum', True), 'contact', 'lift-and-follow-check']
    assert node.attached
    assert mod.PLACE_GAP_M == pytest.approx(.015)
    assert node.vertical is node.guard
    node.set_vacuum(False)
    assert node.suction == 'OFF'
    with pytest.raises(mod.Failure, match='접근'):
        node.set_vacuum(True)


def test_failed_box_follow_still_fails_and_restores_guard():
    mod = fixture_module()
    install(mod)
    node = mod.MoveItPickPlace()
    node.fail_lift = True
    with pytest.raises(mod.Failure, match='did not follow'):
        run_grip(node)
    assert node.vertical is node.guard
    assert not node._virtual_contact_ready


@pytest.mark.parametrize('z', [None, float('nan'), 1.05, .98])
def test_invalid_height_never_turns_on_vacuum(z):
    mod = fixture_module()
    install(mod)
    node = mod.MoveItPickPlace()
    node.z = z
    with pytest.raises(mod.Failure):
        run_grip(node)
    assert node.events == []


def test_non_sim_clock_is_rejected_before_descent():
    mod = fixture_module()
    install(mod)
    node = mod.MoveItPickPlace()
    node.sim_time = False
    with pytest.raises(mod.Failure, match='시뮬레이션'):
        run_grip(node)
    assert node.guard.steps == []


def test_missing_gazebo_world_is_rejected():
    with patch('workcell.virtual_suction.subprocess.run', return_value=SimpleNamespace(stdout='/other/topic\n')):
        with pytest.raises(RuntimeError, match='Gazebo'):
            require_gazebo()
    with patch('workcell.virtual_suction.subprocess.run', return_value=SimpleNamespace(
            stdout='/world/ahead_workcell_v4_2_physical_scale/clock\n')):
        require_gazebo()


def test_no_touch_subscriber_does_not_report_success():
    mod = fixture_module()
    install(mod)
    node = mod.MoveItPickPlace()
    node.create_publisher = lambda *args: SimpleNamespace(get_subscription_count=lambda: 0)
    with pytest.raises(mod.Failure, match='subscriber'):
        run_grip(node)
    assert node.suction == 'OFF'
    assert node.events == []
