#!/usr/bin/env python

import sys
import numpy as np

import roslib; roslib.load_manifest('gt_meka_description')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState


class JointTrajectoryTest():
    def __init__(self, controller='/torso_controller'):
        self.controller = controller
        self.goal_pub = rospy.Publisher(controller+'/command', JointTrajectory)
        self.state_sub = rospy.Subscriber(controller+'/state', JointTrajectoryControllerState, self.state_cb)
        self.joint_names = None

    def state_cb(self, state_msg):
        if self.joint_names is None:
            self.joint_names = state_msg.joint_names

    def up_msg(self):
        jtm = JointTrajectory()
        jtm.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = [1.]*len(self.joint_names)
        jtp.velocities = [0.]*len(self.joint_names)
        jtp.time_from_start = rospy.Duration(5.0)
        jtm.points = [jtp]
        return jtm

    def down_msg(self):
        jtm = JointTrajectory()
        jtm.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.]*len(self.joint_names)
        jtp.velocities = [0.]*len(self.joint_names)
        jtp.time_from_start = rospy.Duration(5.)
        jtm.points = [jtp]
        return jtm

    def random_msg(self):
        jtm = JointTrajectory()
        jtm.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = 2*np.random.random(len(self.joint_names)) - 1
        jtp.velocities = [0.]*len(self.joint_names)
        jtp.time_from_start = rospy.Duration(5.)
        jtm.points = [jtp]
        return jtm

    def run(self):
        while self.joint_names is None:
            print "Waiting for joint state information from %s/state topic" %self.controller
            rospy.sleep(2)
        print "Received joint state information"

        while not rospy.is_shutdown():
            msg = self.down_msg()
            self.goal_pub.publish(msg)
            print "Commanding %s to %s" %(self.controller, msg.points[0].positions)
            rospy.sleep(10)

if __name__=='__main__':
    rospy.init_node('darci_torso_test')
    JTT = JointTrajectoryTest(sys.argv[1])
    JTT.run()

