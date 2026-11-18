#!/usr/bin/python3
# -*- coding:utf-8 -*-
import json
import math
import time
import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType
from ib2_msgs.msg import Navigation

# If the rulebook says one checkpoint is the GOAL and must be visited last,
# put its id here (e.g. GOAL_CHECKPOINT_ID = 7). Leave None otherwise.
GOAL_CHECKPOINT_ID = None

# Motion model used ONLY to rank routes (sample JAXA controller values).
# Only the ratio between them matters for the route order.
V_MAX = 0.20             # m/s   translation speed
W_MAX = 0.436            # rad/s rotation speed
MOTION_PARALLEL = True   # True: leg time = max(move, turn). False: move + turn

HELD_KARP_LIMIT = 13     # up to this many checkpoints -> exact best route
NAV_WAIT_SEC = 3.0       # wait this long for the first pose before planning
MAX_RETRIES = 0          # immediate retries per attempt (second pass handles failures)
SECOND_PASS = True       # retry checkpoints that failed once, at the end
POLL_SEC = 0.05          # how often we check if a move finished


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action clients
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service clients
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Subscribers
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE
        self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)  # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        self.__current_position = None
        self.__current_quaternion = None
        rospy.set_param('/ib2_user/ready', True)  # DO NOT REMOVE THIS LINE

    # ------------------------------------------------------------------
    # Mission notifications
    # ------------------------------------------------------------------
    def __notify_mission_start(self):
        rospy.wait_for_service('/event/mission_start')
        res = self.__mission_start()
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        rospy.wait_for_service('/event/mission_finished')
        res = self.__mission_finished()
        rospy.loginfo('mission_finished result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_checkpoint_passed(self, checkpoint_id):
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        self.__checkpoint_client.wait_for_server()
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_client.send_goal(goal)
        while not rospy.is_shutdown():
            if self.__checkpoint_client.wait_for_result(rospy.Duration(POLL_SEC)):
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result.success
        self.__checkpoint_client.cancel_goal()
        rospy.logwarn('checkpoint_passed action cancelled by node shutdown')
        return False

    # ------------------------------------------------------------------
    # Checkpoint loading
    # ------------------------------------------------------------------
    def __get_checkpoint_id(self, checkpoint):
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in checkpoint:
                return int(checkpoint[key])
        return None

    def __load_all_checkpoints(self):
        """Return a list of dicts: checkpoint_id, position, quaternion."""
        rospy.wait_for_service('/mission/get_checkpoints')
        res = self.__checkpoints()
        rospy.loginfo('get_checkpoints result: success=%s message=%s', res.success, res.message)
        if not res.success:
            return []

        targets = []
        try:
            payload = json.loads(res.message)
            raw = payload.get('checkpoints', payload.get('Checkpoints', []))
        except Exception as e:
            rospy.logwarn('Failed to parse checkpoints payload: %s', e)
            return []

        for cp in raw:
            if not isinstance(cp, dict):
                continue
            cp_id = self.__get_checkpoint_id(cp)
            position = cp.get('position')
            if cp_id is None or not (isinstance(position, list) and len(position) >= 3):
                rospy.logwarn('Skipping malformed checkpoint: %s', cp)
                continue

            quaternion = [0.0, 0.0, 0.0, 1.0]
            attitude = cp.get('attitude')
            if isinstance(attitude, dict):
                q = attitude.get('quaternion')
                if isinstance(q, list) and len(q) >= 4:
                    quaternion = q

            targets.append({
                'checkpoint_id': cp_id,
                'position': [float(position[0]), float(position[1]), float(position[2])],
                'quaternion': [float(quaternion[0]), float(quaternion[1]),
                               float(quaternion[2]), float(quaternion[3])],
            })
        return targets

    # ------------------------------------------------------------------
    # Route planning (cost = estimated TIME, not just distance)
    # ------------------------------------------------------------------
    @staticmethod
    def __dist(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    @staticmethod
    def __angle(q1, q2):
        n1 = math.sqrt(sum(x * x for x in q1)) or 1.0
        n2 = math.sqrt(sum(x * x for x in q2)) or 1.0
        dot = abs(sum(a * b for a, b in zip(q1, q2))) / (n1 * n2)
        return 2.0 * math.acos(min(1.0, dot))

    def __leg_time(self, a, b):
        """Estimated seconds to fly between two poses a, b = (position, quaternion)."""
        t_move = self.__dist(a[0], b[0]) / V_MAX
        t_turn = self.__angle(a[1], b[1]) / W_MAX
        return max(t_move, t_turn) if MOTION_PARALLEL else t_move + t_turn

    @staticmethod
    def __pose(cp):
        return (cp['position'], cp['quaternion'])

    def __route_time(self, start, order):
        total = 0.0
        prev = start
        for cp in order:
            p = self.__pose(cp)
            total += self.__leg_time(prev, p)
            prev = p
        return total

    def __held_karp(self, start, free, goal):
        """Exact fastest order from start through all free checkpoints
        (ending at goal if given). Fast enough for ~13 points."""
        n = len(free)
        poses = [self.__pose(c) for c in free]
        d = [[self.__leg_time(poses[i], poses[j]) for j in range(n)] for i in range(n)]
        d_start = [self.__leg_time(start, poses[j]) for j in range(n)]
        gp = self.__pose(goal) if goal else None
        d_end = [self.__leg_time(poses[j], gp) if gp else 0.0 for j in range(n)]

        dp = {}
        for j in range(n):
            dp[(1 << j, j)] = (d_start[j], -1)
        for mask in range(1, 1 << n):
            for j in range(n):
                if not (mask >> j) & 1 or (mask, j) not in dp:
                    continue
                cost = dp[(mask, j)][0]
                for k in range(n):
                    if (mask >> k) & 1:
                        continue
                    nm = mask | (1 << k)
                    c = cost + d[j][k]
                    if (nm, k) not in dp or c < dp[(nm, k)][0]:
                        dp[(nm, k)] = (c, j)

        full = (1 << n) - 1
        last = min(range(n), key=lambda j: dp[(full, j)][0] + d_end[j])
        order = []
        mask, j = full, last
        while j != -1:
            order.append(free[j])
            mask, j = mask ^ (1 << j), dp[(mask, j)][1]
        order.reverse()
        return order

    def __plan_route(self, targets):
        """Fastest route from the current pose through all checkpoints.
        The goal checkpoint (if configured) is always kept last."""
        goal = None
        free = list(targets)
        if GOAL_CHECKPOINT_ID is not None:
            goal = next((t for t in targets if t['checkpoint_id'] == GOAL_CHECKPOINT_ID), None)
            if goal is not None:
                free = [t for t in targets if t is not goal]

        if self.__current_position is not None and self.__current_quaternion is not None:
            start = (self.__current_position, self.__current_quaternion)
        elif free:
            start = self.__pose(free[0])
            rospy.logwarn('No navigation data, using first checkpoint as route start')
        else:
            start = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

        def full_time(order):
            return self.__route_time(start, order + ([goal] if goal else []))

        if not free:
            best = []
        elif len(free) <= HELD_KARP_LIMIT:
            best = self.__held_karp(start, free, goal)
        else:
            # Too many points for the exact method: nearest neighbour + 2-opt
            remaining = list(free)
            best = []
            pose = start
            while remaining:
                nxt = min(remaining, key=lambda t: self.__leg_time(pose, self.__pose(t)))
                remaining.remove(nxt)
                best.append(nxt)
                pose = self.__pose(nxt)
            improved = True
            while improved:
                improved = False
                for i in range(len(best) - 1):
                    for j in range(i + 1, len(best)):
                        cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                        if full_time(cand) + 1e-9 < full_time(best):
                            best = cand
                            improved = True

        route = best + ([goal] if goal else [])
        rospy.loginfo('planned route (ids): %s, estimated flight time %.1f s',
                      [t['checkpoint_id'] for t in route], full_time(best))
        return route

    def __wait_for_nav(self, timeout):
        """Give the navigation topic a moment so the planner never starts blind."""
        t_end = time.time() + timeout
        while self.__current_position is None and time.time() < t_end and not rospy.is_shutdown():
            rospy.sleep(0.05)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def __move_to_checkpoint(self, checkpoint_target):
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            while not rospy.is_shutdown():
                if self.__command.wait_for_result(rospy.Duration(POLL_SEC)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
            self.__command.cancel_goal()
            rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            self.__action_active = False

    def __visit_checkpoint(self, checkpoint_target):
        for attempt in range(MAX_RETRIES + 1):
            if rospy.is_shutdown():
                return False
            if self.__move_to_checkpoint(checkpoint_target):
                return self.__notify_checkpoint_passed(checkpoint_target['checkpoint_id'])
            rospy.logwarn('move to checkpoint %d failed (attempt %d)',
                          checkpoint_target['checkpoint_id'], attempt + 1)
        return False

    def __fly_route(self, route, label):
        """Visit each checkpoint in order. Returns the list of checkpoints that failed."""
        failed = []
        t_run = time.time()
        for cp in route:
            if rospy.is_shutdown():
                break
            t_leg = time.time()
            ok = self.__visit_checkpoint(cp)
            rospy.loginfo('TIMING %s checkpoint %d: %.1f s (%s)',
                          label, cp['checkpoint_id'], time.time() - t_leg, 'ok' if ok else 'FAILED')
            if not ok:
                failed.append(cp)
        rospy.loginfo('TIMING %s total: %.1f s for %d checkpoints',
                      label, time.time() - t_run, len(route))
        return failed

    # ------------------------------------------------------------------
    # Main flow
    # ------------------------------------------------------------------
    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return

            # USER EDIT AREA: route planning and game-specific flow
            # ***********************************************
            targets = self.__load_all_checkpoints()
            if not targets:
                rospy.logerr('No checkpoints available, finishing mission')
            else:
                self.__wait_for_nav(NAV_WAIT_SEC)
                t_plan = time.time()
                route = self.__plan_route(targets)
                rospy.loginfo('planning took %.3f s', time.time() - t_plan)

                failed = self.__fly_route(route, 'pass1')
                if failed and SECOND_PASS and not rospy.is_shutdown():
                    rospy.logwarn('retrying %d failed checkpoint(s): %s',
                                  len(failed), [c['checkpoint_id'] for c in failed])
                    self.__fly_route(failed, 'pass2')
            # ***********************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __callback_nav(self, data):
        p = data.pose.pose.position
        q = data.pose.pose.orientation
        self.__current_position = [p.x, p.y, p.z]
        self.__current_quaternion = [q.x, q.y, q.z, q.w]

    def __stop_processing(self, request):
        try:
            if self.__action_active:
                self.__command.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()