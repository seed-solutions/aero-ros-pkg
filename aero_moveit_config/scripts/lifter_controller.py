#!/usr/bin/env python

import rospy
import time

from aero_startup.srv import AeroTorsoController

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult,
)

import actionlib


class FollowAction(object):
    _feedback = FollowJointTrajectoryActionFeedback()
    _result   = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self._joints = ["virtual_lifter_x_joint", "virtual_lifter_z_joint"]
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb)
        self._as.start()


    def execute_cb(self, goal):
        z_only = False
        if len(goal.trajectory.joint_names) == 1:
            goal.trajectory.joint_names.append("virtual_lifter_x_joint")
            z_only = True
        if not self.equalJoints(goal.trajectory.joint_names):
            rospy.logerr("inaccurate joint names received")
            return

        print "execute" + rospy.get_name()
        print "joint_names"
        print goal.trajectory.joint_names
        print goal.trajectory.points[-1].positions[0]
        try:
            srv = rospy.ServiceProxy("aero_torso_controller" ,AeroTorsoController)
            if z_only:
                x = 0
            else:
                x = goal.trajectory.points[-1].positions[goal.trajectory.joint_names.index("virtual_lifter_x_joint")] * 1000
            
            z = goal.trajectory.points[-1].positions[goal.trajectory.joint_names.index("virtual_lifter_z_joint")] * 1000
            res = srv(x,z,"world")
            result = FollowJointTrajectoryResult()
            if res.status == "success":
                time.sleep(res.time_sec)
                result.error_code = 0
                self._as.set_succeeded()
                rospy.loginfo('%s: Succeeded' % self._action_name)
                return
            #error syori wo kaku
            self._as.set_aborted()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self._as.set_aborted()


    def equalJoints(self, joints):
        if len(self._joints) != len(joints):
            return False
        for j in joints:
            if not (j in self._joints):
                return False
        return True


if __name__ == '__main__':
    rospy.init_node('lifter_controller')
    FollowAction(rospy.get_name() + "/follow_joint_trajectory")
    rospy.spin()
