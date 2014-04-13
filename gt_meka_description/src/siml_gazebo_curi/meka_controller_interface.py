#!/usr/bin/env python

import roslib; roslib.load_manifest('gt_meka_description')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from m3ctrl_msgs.msg import M3JointCmd
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
import numpy as np

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class MekaControllerConverter():
    '''
    This node is entirely a stopgap measure so that when we write controllers
    for Curi, it will be the same effect in Gazebo because currently
    Gazebo motor transmission files are Pr2 files that use a different
    naming convention
    '''

    # Some constants that are defined in the C headers duplicated here
    (RIGHT_ARM,LEFT_ARM,TORSO,
     HEAD,RIGHT_HAND,LEFT_HAND) = range(6)

    (JOINT_MODE_ROS_OFF,
     JOINT_MODE_ROS_THETA,
     JOINT_MODE_ROS_THETA_GC,
     JOINT_MODE_ROS_TORQUE_GC,
     JOINT_MODE_ROS_POSE) = range(5)

    (SMOOTHING_MODE_OFF,
     SMOOTHING_MODE_SLEW,
     SMOOTHING_MODE_MIN_JERK) = range(3) 

    def __init__(self):

        # Setup the joint controllers we want to convert
        #self.joint_controllers = {MekaControllerConverter.RIGHT_ARM: '/r_arm_controller/command',
        #                          MekaControllerConverter.LEFT_ARM: '/l_arm_controller/command',
        #                          MekaControllerConverter.HEAD: '/head_controller/command',
        #                          MekaControllerConverter.RIGHT_HAND: '/r_hand_controller/command',
        #                          MekaControllerConverter.LEFT_HAND: '/l_hand_controller/command'}

        self.joint_controllers = {MekaControllerConverter.RIGHT_ARM: '/r_arm_controller/',
                                  MekaControllerConverter.LEFT_ARM: '/l_arm_controller/',
                                  MekaControllerConverter.HEAD: '/head_controller/',
                                  MekaControllerConverter.RIGHT_HAND: '/r_hand_controller/',
                                  MekaControllerConverter.LEFT_HAND: '/l_hand_controller/'}
        # Setup the humanoid state status
        # Order is: right_arm, left_arm, head, right_hand, left_hand
        # Also setup all of the publishers for each controller
        self.joint_names = []
        self.publishers = dict()

        for part in self.joint_controllers:
            controller = self.joint_controllers[part]
            self.joint_names.extend(get_param(controller+'joints', ''))
            self.publishers[controller] = rospy.Publisher(controller+'command', JointTrajectory)

        print self.joint_names
        self.positions = [0.0]*len(self.joint_names)
        self.velocities = [0.0]*len(self.joint_names)
        self.effort = [0.0]*len(self.joint_names)
        
        rospy.loginfo("Setting up subscribers and publishers")
        # Setup subscribers to listen to the commands
        self.humanoid_command_sub = rospy.Subscriber('/humanoid_command', M3JointCmd, self.humanoidCallback)          
        self.zlift_command_sub = rospy.Subscriber('/zlift_command', M3JointCmd, self.zliftCallback)          
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)          

        # Replicate the humanoid_state 
        self.humanoid_state_pub = rospy.Publisher('/humanoid_state', JointState)          

        rospy.loginfo("Done Init")


    def humanoidCallback(self, msg):
        '''
        When callback receives a humanoid control command
        convert it into the proper controllers (arms, head, etc.)
        for gazebo
        '''
        trajectory_store = dict()

        # Initialize the dictionary for all of the parts
        for part in self.joint_controllers:
            jtm = JointTrajectory()
            jtp = JointTrajectoryPoint()
            jtp.time_from_start = rospy.Duration(1.0)
            jtp.positions = []
            jtm.points = [jtp] 
            trajectory_store[part] = jtm
            
        # Go through the message and fill in each command separately
        for i in range(len(msg.chain)):
          
            chain_num = ord(msg.chain[i])
            jtm = trajectory_store[chain_num]
            jtm.joint_names = get_param(self.joint_controllers[chain_num]+'joints','')
            jtm.points[0].positions.append(msg.position[i])
            trajectory_store[chain_num] = jtm

        # Now go through and publish each controller
        for part in self.joint_controllers:

            # Get the actual controller name
            controller = self.joint_controllers[part]
            pub = self.publishers[controller]
            pub.publish(trajectory_store[part])


    def zliftCallback(self, msg):
        print 'In callback' 


    def fillJointCommand(self, ):
        print 'In callback' 

    # Go through each of the joints and populate
    def joint_state_cb(self, joint_states):
        
        for i in range(len(self.joint_names)):
            for j in range(len(joint_states.name)):
                if self.joint_names[i] == joint_states.name[j]:
                    self.positions[i] = joint_states.position[j]
                    self.velocities[i] = joint_states.velocity[j]
                    self.effort[i] = joint_states.effort[j]

        # Publish the m3 humanoid state
        self.humanoid_state_pub.publish(JointState(joint_states.header, self.joint_names, self.positions, self.velocities, self.effort))

if __name__=='__main__':
    rospy.init_node('MekaController2Gazebo')
    rospy.loginfo("Starting up Meka Controller Converter Node")
    MekaControllerConverter()
    rospy.spin()


