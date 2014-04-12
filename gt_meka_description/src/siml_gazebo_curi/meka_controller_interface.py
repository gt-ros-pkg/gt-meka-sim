#!/usr/bin/env python

import roslib; roslib.load_manifest('gt_meka_description')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from m3ctrl_msgs.msg import M3JointCmd
from sensor_msgs.msg import JointState

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


        # Setup some constants
        self.RIGHT_ARM_CHAIN_SIZE = 7
        self.LEFT_ARM_CHAIN_SIZE = 7
        self.RIGHT_HAND_CHAIN_SIZE = 5
        self.LEFT_HAND_CHAIN_SIZE = 5
        self.HEAD_CHAIN_SIZE = 12
        self.HUMANOID_CHAIN_SIZE = self.RIGHT_ARM_CHAIN_SIZE + self.RIGHT_HAND_CHAIN_SIZE + self.LEFT_ARM_CHAIN_SIZE + self.LEFT_HAND_CHAIN_SIZE + self.HEAD_CHAIN_SIZE

        # Setup the humanoid state status
        # Order is: right_arm, left_arm, head, right_hand, left_hand
        self.joint_names = []
        self.joint_names.extend(get_param('/r_arm_controller/joints', ''))
        self.joint_names.extend(get_param('/l_arm_controller/joints', ''))
        self.joint_names.extend(get_param('/head_controller/joints', ''))
        self.joint_names.extend(get_param('/r_hand_controller/joints', ''))
        self.joint_names.extend(get_param('/l_hand_controller/joints', ''))

        self.positions = [0.0]*len(self.joint_names)
        self.velocities = [0.0]*len(self.joint_names)
        self.effort = [0.0]*len(self.joint_names)

        rospy.loginfo("Setting up subscribers and publishers")
        # Setup subscribers to listen to the commands
        self.humanoid_command_sub = rospy.Subscriber('/humanoid_command', M3JointCmd, self.humanoidCallback)          
        self.zlift_command_sub = rospy.Subscriber('/zlift_command', M3JointCmd, self.zliftCallback)          
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)          

        # Setup publisher to republish the commands
        self.l_arm_command_pub = rospy.Publisher('/l_arm_controller/command', JointTrajectory)          
        self.r_arm_command_pub = rospy.Publisher('/r_arm_controller/command', JointTrajectory)          
        self.head_command_pub = rospy.Publisher('/head_controller/command', JointTrajectory)          
        self.zlift_command_pub = rospy.Publisher('/torso_controller/command', JointTrajectory)          
       
        # Replicate the humanoid_state 
        self.humanoid_state_pub = rospy.Publisher('/humanoid_state', JointState)          

        rospy.loginfo("Done Init")


    def humanoidCallback(self, msg):
        '''
        message information:
        uint8[] chain //Which kinematic chain
        int16[] chain_idx //Which joint of the chain 
        float32[] stiffness //Joint stiffness in THETA_GC mode (0-1.0)
        float32[] velocity  //Joint velocity (rad/s) when using a smoothing filter
        float32[] position  //Desired joint position (rad)
        uint8[] control_mode //Desired control mode
        uint8[] smoothing_mode //Desired smoothing mode
        '''
        print 'In callback' 



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

'''
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
'''
if __name__=='__main__':
    rospy.init_node('MekaController2Gazebo')
    rospy.loginfo("Starting up Meka Controller Converter Node")
    MekaControllerConverter()
    rospy.spin()


