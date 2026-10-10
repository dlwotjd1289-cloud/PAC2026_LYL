"""Regression: a completed box at PICK must no longer hold the conveyor.

This checks only belt scheduling. It does not assume suction succeeded.
"""
from types import SimpleNamespace
from unittest.mock import Mock
from workcell import lyl_cell_runner as module


def test_completed_first_box_does_not_block_second_arrival():
    r = module.Runner.__new__(module.Runner)
    r.stop = False
    r.g = dict(scale_centre_x=-5.985, stopper_face_x=-0.845,
               roller_top_z=0.895, conveyor_y=1.20, inlet_x=-6.635)
    r.c = dict(max_queue=3, settle_s=1.0, weigh_window_s=0.5)
    r.v, r.omega, r.pitch_m = 0.30, 1.20, 0.20
    r.cfg = {'perception': {'dent_m': 0.025}}
    r.run = SimpleNamespace(log=Mock(), event=Mock())
    size = SimpleNamespace(x=0.30, y=0.25, z=0.20)

    def box(number, *, done):
        return SimpleNamespace(name=f'box_{number:02d}', done=done, picked=False,
                               field=SimpleNamespace(truth=SimpleNamespace(size=size), damaged=False),
                               weighed=done, at_stopper_logged=done,
                               last_x=None, still_since=None)

    b1, b2 = box(1, done=True), box(2, done=False)
    r.boxes = [b1, b2]
    r.spawn = Mock()

    class Cell:
        clock = 2.0

        def __init__(self):
            self.roller_speeds = []
            self.ticks = 0

        def spin(self, seconds):
            self.clock += seconds
            if seconds == 0.05:
                self.ticks += 1
                if self.ticks >= 4:
                    r.stop = True

        def force_mean(self, since):
            return (98.1, 20)

        def pose(self, name):
            if name == 'box_01':
                # Model pose may remain in the pickup corridor after placement.
                return (-0.94, 1.20, 1.0, 0., 0., 0., 1., 1.)
            return (-6.30, 1.20, 1.0, 0., 0., 0., 1., 1.)

        def roller(self, omega):
            self.roller_speeds.append(omega)

    r.cell = Cell()
    r.conveyor_loop()

    assert r.spawn.call_count >= 2, 'Second arrival must be eligible for spawning'
    assert r.cell.roller_speeds[:-1] and all(
        speed == r.omega for speed in r.cell.roller_speeds[:-1]
    ), 'A completed box must not keep the conveyor stopped'
