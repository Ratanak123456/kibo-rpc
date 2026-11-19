"""Platform controller (/ctl/command action): move to an absolute pose."""
import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType

from kibo import config
from kibo.ros_io import log

SUCCESS = CtlCommandResult.TERMINATE_SUCCESS
ABORTED = CtlCommandResult.TERMINATE_ABORTED


class CtlClient(object):

    def __init__(self):
        self._client = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.active = False

    def wait_ready(self):
        if not self._client.wait_for_server(rospy.Duration(config.WAIT_SERVICE_TIMEOUT)):
            log.warn('/ctl/command server not ready yet')

    def send(self, position, quat):
        target = PoseStamped()
        target.header.frame_id = 'iss_body'
        target.header.stamp = rospy.Time.now()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        target.pose.orientation.x, target.pose.orientation.y = quat[0], quat[1]
        target.pose.orientation.z, target.pose.orientation.w = quat[2], quat[3]
        goal = CtlCommandGoal(target=target,
                              type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))
        self.active = True
        self._client.send_goal(goal)

    def poll(self):
        """None while moving, else the CtlCommandResult type."""
        if not self.active:
            return SUCCESS
        if self._client.wait_for_result(rospy.Duration(0.001)):
            self.active = False
            res = self._client.get_result()
            return res.type if res is not None else ABORTED
        return None

    def cancel(self, wait=True):
        if not self.active:
            return
        try:
            self._client.cancel_goal()
            if wait:
                self._client.wait_for_result(rospy.Duration(2.0))
        except Exception as exc:  # noqa: BLE001 - best effort
            log.warn('cancel failed: %s', exc)
        if wait:
            self.active = False
