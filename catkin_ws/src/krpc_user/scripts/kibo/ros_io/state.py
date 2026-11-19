"""Robot state from the platform: navigation and ctl status."""
import math
import threading

import rospy
from ib2_msgs.msg import CtlStatus, Navigation

from kibo.core.geometry import q_normalize


class NavSample(object):
    """One /sensor_fusion/navigation message (dock frame; angvel in body frame)."""
    __slots__ = ('stamp', 'pos', 'vel', 'quat', 'angvel', 'speed', 'ang_speed')

    def __init__(self, stamp, pos, vel, quat, angvel):
        self.stamp = stamp          # [s] measurement time (arrives ~0.1 s late)
        self.pos = pos
        self.vel = vel
        self.quat = quat
        self.angvel = angvel
        self.speed = math.sqrt(sum(c * c for c in vel))
        self.ang_speed = math.sqrt(sum(c * c for c in angvel))


class RobotState(object):

    def __init__(self):
        self.nav = None             # latest NavSample
        self.ctl_type = None        # latest CtlStatusType value
        self._fresh = threading.Event()
        self._nav_sub = rospy.Subscriber('/sensor_fusion/navigation', Navigation,
                                         self._on_nav, queue_size=1)
        self._ctl_sub = rospy.Subscriber('/ctl/status', CtlStatus, self._on_ctl, queue_size=1)

    def _on_nav(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        lin = msg.twist.linear
        ang = msg.twist.angular
        try:
            stamp = msg.pose.header.stamp.to_sec()
        except Exception:  # noqa: BLE001 - missing header: stamp unknown
            stamp = None
        self.nav = NavSample(stamp, [p.x, p.y, p.z], [lin.x, lin.y, lin.z],
                             q_normalize([o.x, o.y, o.z, o.w]), [ang.x, ang.y, ang.z])
        self._fresh.set()

    def _on_ctl(self, msg):
        try:
            self.ctl_type = int(msg.type.type)
        except Exception:  # noqa: BLE001 - malformed status: keep last value
            pass

    def wait_fresh_nav(self, timeout):
        """Wait (wall time) for a nav sample newer than the last call."""
        got = self._fresh.wait(timeout)
        self._fresh.clear()
        return got
