#!/usr/bin/python3
# -*- coding:utf-8 -*-
"""Kibo-RPC 7th - user program entry point (team I-ORBIT).

Kept to the platform template: the start subscriber, the stop service and
the ready parameter must stay exactly as provided. All mission logic lives in
the `kibo` package next to this file (see kibo/__init__.py for the layout).
"""
import os
import sys

# catkin runs this file through a relay script in devel/lib/krpc_user, so the
# script directory is not on sys.path by default: add it to import `kibo`.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy  # noqa: E402
from platform_msgs.msg import UserLogic  # noqa: E402
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse  # noqa: E402

from kibo.mission_runner import MissionRunner  # noqa: E402


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')
        self.__runner = MissionRunner()
        self.__started = False

        self.__subscriber_start = rospy.Subscriber(
            '/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE
        self.__stop_processing_server = rospy.Service(
            '/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)  # DO NOT REMOVE THIS LINE

        rospy.set_param('/ib2_user/ready', True)  # DO NOT REMOVE THIS LINE

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True
        rospy.loginfo('start %s', msg)
        self.__runner.run()

    def __stop_processing(self, request):
        try:
            self.__runner.stop()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:  # noqa: BLE001
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)


if __name__ == '__main__':
    UserTemplate()
    rospy.spin()
