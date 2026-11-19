"""Simulation clock and mission timer.

The judge scores in SIMULATION time and the web simulator runs at ~0.35x
real time, so wall-clock time must never be used for the mission budget.
/clock is used when published (simulator); otherwise ROS time (real IB2).
"""
import time

import rospy
from rosgraph_msgs.msg import Clock

from kibo import config


def ros_ok():
    return not rospy.is_shutdown()


class SimClock(object):

    def __init__(self):
        self._sim = None
        self._sub = rospy.Subscriber('/clock', Clock, self._on_clock, queue_size=1)

    def _on_clock(self, msg):
        self._sim = msg.clock.to_sec()

    @property
    def source(self):
        return '/clock' if self._sim is not None else 'rospy'

    def now(self):
        return self._sim if self._sim is not None else rospy.get_time()

    def sleep(self, sec, alive):
        """Sleep sec of sim time; returns early if alive() turns False or the
        clock stalls (bounded by wall time)."""
        start_sim, start_wall = self.now(), time.time()
        while alive():
            if self.now() - start_sim >= sec:
                return
            if time.time() - start_wall > max(1.0, sec * 6.0):
                return
            time.sleep(0.01)


class MissionTimer(object):
    """Mission time budget, measured from mission_start in sim time."""

    def __init__(self, clock, limit=config.TIME_LIMIT_SEC, guard=config.FINISH_GUARD_SEC):
        self.clock = clock
        self.limit = limit
        self.guard = guard
        self.t0 = None

    def start(self, t0):
        self.t0 = t0

    def elapsed(self):
        return 0.0 if self.t0 is None else self.clock.now() - self.t0

    def remaining(self):
        return self.limit - self.elapsed()

    def deadline_hit(self):
        """True when mission_finished must be sent now."""
        return self.remaining() <= self.guard
