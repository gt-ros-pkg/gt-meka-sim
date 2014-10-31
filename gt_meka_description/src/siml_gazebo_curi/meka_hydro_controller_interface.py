#!/usr/bin/env python
import roslib; roslib.load_manifest('gt_meka_description')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
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
    ZLIFT = 6

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

        self.joint_controllers = {MekaControllerConverter.RIGHT_ARM: '/r_arm_controller/',
                                  MekaControllerConverter.LEFT_ARM: '/l_arm_controller/',
                                  MekaControllerConverter.HEAD: '/head_controller/',
                                  MekaControllerConverter.RIGHT_HAND: '/r_hand_controller/',
                                  MekaControllerConverter.LEFT_HAND: '/l_hand_controller/'}

        # Need to define these manually because URDF doesn't reflect real robot
        # The real robot does not have access to all finger joints but gazebo requires
        # all joints to be defined to not explode
        self.right_hand_joint_names = ['right_hand_j0','right_hand_j1','right_hand_j2','right_hand_j3','right_hand_j4']
        self.left_hand_joint_names = ['left_hand_j0','left_hand_j1','left_hand_j2','left_hand_j3','left_hand_j4']

        # Setup the humanoid state status
        # Order is: right_arm, left_arm, head, right_hand, left_hand
        # Also setup all of the publishers for each controller
        self.joint_names = []
        self.publishers = dict()

        for part in self.joint_controllers:
            controller = self.joint_controllers[part]

            # Specifically to take care of the extra finger joints
            if part == MekaControllerConverter.RIGHT_HAND:
                self.joint_names.extend(self.right_hand_joint_names)
            elif part == MekaControllerConverter.LEFT_HAND:
                self.joint_names.extend(self.left_hand_joint_names)
            else:
                self.joint_names.extend(get_param(controller+'joints', ''))

            self.publishers[controller] = rospy.Publisher(controller+'command', JointTrajectory)
        
        self.positions = [0.0]*len(self.joint_names)
        self.velocities = [0.0]*len(self.joint_names)
        self.effort = [0.0]*len(self.joint_names)
       
        # Specific for the zlift
        self.zlift_pub = rospy.Publisher('/torso_controller/command', JointTrajectory)          
        self.zlift_joint_names = get_param('/torso_controller/joints', '')          

        self.zlift_joint_names = ['torso_lift_joint']
        self.zlift_positions = [0.0]*len(self.zlift_joint_names)
        self.zlift_velocities = [0.0]*len(self.zlift_joint_names)
        self.zlift_effort = [0.0]*len(self.zlift_joint_names)

        rospy.loginfo("Setting up subscribers and publishers")
        # Setup subscribers to listen to the commands
        self.humanoid_command_sub = rospy.Subscriber('/humanoid_command', M3JointCmd, self.humanoidCallback)          
        self.zlift_command_sub = rospy.Subscriber('/zlift_command', M3JointCmd, self.zliftCallback)          
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)          

        # Replicate the humanoid_state 
        self.humanoid_state_pub = rospy.Publisher('/humanoid_state', JointState)          
        self.zlift_state_pub = rospy.Publisher('/zlift_state', JointState)          

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
           
        chain_values = []
        # Go through the message and fill in each command separately
        for i in range(len(msg.chain)):
            chain_num = ord(msg.chain[i])
            chain_values.append(chain_num)
            jtm = trajectory_store[chain_num]
            jtm.points[0].positions.append(msg.position[i])
            trajectory_store[chain_num] = jtm

        # Figure out the unique joints that were called
        sent_controllers = np.unique(chain_values)

        # Go through the actual unique commands and populate the fields
        for chain_cmd in sent_controllers:
            jtm = trajectory_store[chain_cmd]
            jtm.joint_names = get_param(self.joint_controllers[chain_cmd]+'joints','')

            # Special case for fingers - add zeros to command
            if chain_cmd == MekaControllerConverter.RIGHT_HAND or chain_cmd == MekaControllerConverter.LEFT_HAND:
                jtm.points[0].positions.extend([0.0]*(len(jtm.joint_names)-len(jtm.points[0].positions)))

            trajectory_store[chain_cmd] = jtm

        # Only go through the controllers that were actually sent
        for part in sent_controllers:

            # Get the actual controller name
            controller = self.joint_controllers[part]
            pub = self.publishers[controller]
            pub.publish(trajectory_store[part])


    def zliftCallback(self, msg):

        # Setup the joint trajectory
        jtm = JointTrajectory()
        jtp = JointTrajectoryPoint()
        jtm.joint_names = self.zlift_joint_names
        jtp.time_from_start = rospy.Duration(0.0)
        jtp.positions = [msg.position[0]*0.001] #Convert to meters
        jtm.points = [jtp] 
        
        self.zlift_pub.publish(jtm)

    # Go through each of the joints and populate
    # Populates the humanoid_state specifically
    def joint_state_cb(self, joint_states):
   
        for i in range(len(self.joint_names)):
            for j in range(len(joint_states.name)):
                if self.joint_names[i] == joint_states.name[j]:
                    self.positions[i] = joint_states.position[j]
                    self.velocities[i] = joint_states.velocity[j]
                    self.effort[i] = joint_states.effort[j]
                if 'torso_lift_joint' == joint_states.name[j]:
                    self.zlift_positions[0] = joint_states.position[j]
                    self.zlift_velocities[0] = joint_states.velocity[j]
                    self.zlift_effort[0] = joint_states.effort[j]

        # Publish the m3 humanoid state
        self.humanoid_state_pub.publish(JointState(joint_states.header, self.joint_names, self.positions, self.velocities, self.effort))

        self.zlift_state_pub.publish(JointState(joint_states.header, self.zlift_joint_names, self.zlift_positions, self.zlift_velocities, self.zlift_effort))

if __name__=='__main__':
    rospy.init_node('MekaController2Gazebo')
    rospy.loginfo("Starting up Meka Controller Converter Node")
    MekaControllerConverter()
    rospy.spin()


