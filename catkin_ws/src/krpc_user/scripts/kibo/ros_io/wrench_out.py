"""/ctl/wrench output: body force/torque straight to the fan allocator (fsm).

Only valid while the platform ctl is idle (STAND_BY), because ctl publishes
its own wrench whenever it is active. Once closed, only zeros are sent, so a
late control tick can never re-enable the fans after a stop or hand-over.
"""
import threading

import rospy
from geometry_msgs.msg import WrenchStamped

_ZERO = (0.0, 0.0, 0.0)


class WrenchOutput(object):

    def __init__(self):
        self._pub = rospy.Publisher('/ctl/wrench', WrenchStamped, queue_size=1)
        self._lock = threading.Lock()
        self.closed = False
        self.used = False

    def publish(self, force, torque):
        with self._lock:
            if self.closed:
                force, torque = _ZERO, _ZERO
            msg = WrenchStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'body'
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque
            self._pub.publish(msg)
            self.used = True

    def close(self):
        """Zero the fans and latch closed (idempotent)."""
        with self._lock:
            self.closed = True
        for _ in range(3):
            self.publish(_ZERO, _ZERO)
