"""Kibo-RPC game API: mission start/finish, checkpoint list and reports."""
import threading
import time

import actionlib
import rospy
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from krpc_ib2_msgs.srv import CheckPoints, MissionFinished, MissionStart

from kibo import config
from kibo.core.mission import parse_mission
from kibo.ros_io import log

_SERVICES = ('/event/mission_start', '/event/mission_finished', '/mission/get_checkpoints')


class GameApi(object):

    def __init__(self):
        self._start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self._finish = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self._list = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)
        self._cp = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)
        self._cp_active = False
        self._finish_lock = threading.Lock()
        self.started = False
        self.finished = False

    def wait_ready(self):
        for name in _SERVICES:
            try:
                rospy.wait_for_service(name, timeout=config.WAIT_SERVICE_TIMEOUT)
            except rospy.ROSException:
                log.warn('service %s not ready yet', name)
        if not self._cp.wait_for_server(rospy.Duration(config.WAIT_SERVICE_TIMEOUT)):
            log.warn('/event/checkpoint_passed server not ready yet')

    def mission_start(self):
        try:
            res = self._start()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            log.error('mission_start failed: %s', exc)
            return False
        log.info('mission_start result: success=%s message=%s', res.success, res.message)
        self.started = bool(res.success)
        return self.started

    def mission_finished(self, elapsed=None):
        """Report the end of the mission. Safe to call many times / from any thread."""
        with self._finish_lock:
            if self.finished or not self.started:
                return False
            self.finished = True
        try:
            res = self._finish()
            log.info('mission_finished result: success=%s message=%s (elapsed %s)',
                     res.success, res.message,
                     '%.1fs' % elapsed if elapsed is not None else '?')
            return res.success
        except (rospy.ROSException, rospy.ServiceException) as exc:
            log.error('mission_finished failed: %s', exc)
            return False

    def load_mission(self, retry_sleep):
        """Mission from /mission/get_checkpoints, or None. retry_sleep(sec) between tries."""
        for attempt in range(config.LOAD_RETRIES):
            try:
                res = self._list()
                if not res.success:
                    log.warn('get_checkpoints returned success=False')
                    continue
                mission = parse_mission(res.message)
                log.info('loaded %d checkpoints, goal id=%s',
                         len(mission.checkpoints), mission.goal_id)
                return mission
            except Exception as exc:  # noqa: BLE001 - retry on any failure
                log.warn('get_checkpoints attempt %d failed: %s', attempt + 1, exc)
                retry_sleep(0.3)
        return None

    def checkpoint_passed(self, checkpoint_id):
        """Ask the judge to evaluate a checkpoint NOW. One shot: the judge
        ignores repeats, so only call this once the pose is verified."""
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self._cp_active = True
        try:
            self._cp.send_goal(goal)
            wall_deadline = time.time() + config.CHECKPOINT_CALL_TIMEOUT
            while not rospy.is_shutdown() and time.time() < wall_deadline:
                if self._cp.wait_for_result(rospy.Duration(0.1)):
                    result = self._cp.get_result()
                    ok = bool(result and result.success)
                    log.info('checkpoint_passed(%d): success=%s', checkpoint_id, ok)
                    return ok
            self._cp.cancel_goal()
            log.warn('checkpoint_passed(%d) timed out', checkpoint_id)
            return False
        finally:
            self._cp_active = False

    def cancel_pending(self):
        if self._cp_active:
            self._cp.cancel_goal()
