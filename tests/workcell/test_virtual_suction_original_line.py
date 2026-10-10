"""The original V4.6 MoveIt executor uses line(), not vertical; no ROS needed."""
from types import SimpleNamespace

import pytest

from workcell.virtual_suction import install


def test_original_line_interface_keeps_other_movements_and_restores_method():
    class Failure(RuntimeError):
        pass

    class Original:
        def __init__(self):
            self.suction = 'OFF'
            self.calls = []
            self.touch_count = 0

        def get_parameter(self, name):
            return SimpleNamespace(value=True)

        def tcp_z(self):
            return 1.01

        def fresh_box(self):
            return ([0, 0, 0.5], None, 0)

        def create_publisher(self, msg_type, name, queue_size):
            assert name == '/pac/suction/touched'
            def publish(value):
                self.touch_count += 1
                self.suction = 'GRIPPED'
            return SimpleNamespace(get_subscription_count=lambda: 1, publish=publish)

        def spin_for(self, seconds):
            pass

        def wait(self, predicate, timeout, what):
            if not predicate():
                raise Failure(what)

        def set_vacuum(self, on):
            self.suction = 'SEARCHING' if on else 'OFF'

        def line(self, xyz, yaw, what, avoid_collisions=True, speed=.1):
            self.calls.append(what)

        def grip(self, b, yaw, top, size, q, at_pick):
            self.line([0, 0, 1.20], yaw, 'lower to approach height')
            self.line([0, 0, 1.01], yaw, 'descend to box (collision-checked)')
            self.line([0, 0, 0.999], yaw, 'press cup on box', avoid_collisions=False)
            self.set_vacuum(True)
            self.wait(lambda: self.suction == 'GRIPPED', 5, 'GRIPPED')
            self.line([0, 0, 1.20], yaw, 'lift box straight up')
            return .20

    m = SimpleNamespace(MoveItPickPlace=Original, Bool=SimpleNamespace,
                        Failure=Failure, PLACE_GAP_M=.005, CHECKED_GAP_M=.010)
    install(m)
    node = Original()
    assert not hasattr(node, 'vertical')
    unpatched_line = node.line.__func__
    assert node.grip([0, 0, .5], 0, 1., [1, 1, 1], None, True) == .20
    assert node.calls == ['lower to approach height',
                          'descend to box (collision-checked)', 'lift box straight up']
    assert node.touch_count == 1
    assert m.PLACE_GAP_M == pytest.approx(.015)
    assert node.line.__func__ is unpatched_line
    assert not node._virtual_contact_ready
    with pytest.raises(Failure, match='접근'):
        node.set_vacuum(True)
