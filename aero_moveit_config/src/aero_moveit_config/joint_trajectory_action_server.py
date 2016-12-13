#!/usr/bin/env python

import rospy

#from dynamic_reconfigure.server import Server

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

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
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb)
        self._as.start()
        controllers = rospy.get_param(rospy.get_name() + "/controller_list")
        self._joints = []
        for cont in controllers:
            if "/{}".format(cont["name"]) != rospy.get_name():
                continue
            self._joints = cont["joints"]
        if not self._joints:
            rospy.logerr("{} not found in controllers list".format(rospy.get_name()))
            exit()
        self._pub_command = rospy.Publisher("/aero_controller/command", JointTrajectory, queue_size=1)
        self._sub_states = rospy.Subscriber("/aero_joint_states", JointState, self.states_cb)
        self._active_flag = False
        self._finishable = False


    def execute_cb(self, goal):
        if not self.equalJoints(goal.trajectory.joint_names):
            rospy.logerr("inaccurate joint names received")
            return

        if self._active_flag:
            nans = JointTrajectory()
            nans.header = goal.trajectory.header
            nans.joint_names = goal.trajectory.joint_names
            nans.points = JointTrajectoryPoint()
            nans.points.positions = [float("nan")] * len(nans.joint_names)


        print "execute" + rospy.get_name()
        self._finishable = False
        command = JointTrajectory()
        command.header = goal.trajectory.header
        command.joint_names = goal.trajectory.joint_names
        command.points = [goal.trajectory.points[-1], ]
        self._active_goal = goal
        self._active_command = command
        self._active_end_time = rospy.get_rostime() + goal.trajectory.points[-1].time_from_start + goal.goal_time_tolerance + rospy.Duration(2, 0) #2.0 sec is nantonaku
        self._pub_command.publish(command)    
        self._active_flag = True
        self._execute_state = True

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._finishable:
                break
            r.sleep()

        self._active_flag = False
        if not self._execute_state:
            self._result.error_string = "execution timeout"
            self._as.set_aborted(self._result)
            return
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._result.error_string = "execution succeeded"
        self._result.error_code = 0
        self._as.set_succeeded(self._result)



    def states_cb(self, states):
        if not self._active_flag:
            return

        if rospy.get_rostime() > self._active_end_time:
            self._finishable = True
            self._execute_state = False
            return

        all_in_tole = True
        for i, joint in enumerate(self._active_goal.trajectory.joint_names):
            j = states.name.index(joint)
            if abs(states.position[j] - self._active_goal.trajectory.points[-1].positions[i]) > 0.05:
                all_in_tole = False
                rospy.loginfo('%s: timeout' % self._action_name)

                break
        if all_in_tole:
            rospy.loginfo('%s: Inside tolerance' % self._action_name)
            self._finishable = True
            return
        return



    def equalJoints(self, joints):
        print self._joints
        if len(self._joints) != len(joints):
            return False
        for j in joints:
            if not (j in self._joints):
                return False
        return True


if __name__ == '__main__':
    rospy.init_node('joint_trajectory_action_server')
    FollowAction(rospy.get_name() + "/follow_joint_trajectory")
    rospy.spin()
