#!/usr/bin/env python

import roslib; roslib.load_manifest('hrl_gazebo_darci')
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
        jtp.positions = [0.6]*len(self.joint_names)
        jtp.time_from_start = rospy.Duration(1.0)
        jtm.points = [jtp]
        return jtm

    def run(self):
        while self.joint_names is None:
            print "Waiting for joint state information from %s/state topic" %self.controller
            rospy.sleep(2)
        print "Received joint state information. Sending torso to default position (0.6m)"
        up_msg = self.up_msg()
        self.goal_pub.publish(up_msg)

if __name__=='__main__':
    rospy.init_node('darci_torso_setup')
    JTT = JointTrajectoryTest()
    JTT.run()
