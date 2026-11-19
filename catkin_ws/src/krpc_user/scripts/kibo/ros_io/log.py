"""Logging through rosout (ends up in user_program_output.log)."""
import rospy


def info(fmt, *args):
    rospy.loginfo(fmt, *args)


def warn(fmt, *args):
    rospy.logwarn(fmt, *args)


def error(fmt, *args):
    rospy.logerr(fmt, *args)
