#! /usr/bin/python
# -*- coding: utf-8 -*-


#M3 -- Meka Robotics Robot Components
#Copyright (C) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.

import time
import os
import roslib; roslib.load_manifest('gt_meka_description')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def shm_humanoid(joint_states):
    for i in range(len(joints)):
      for j in range(len(joint_states.name)):
	if joints[i] == joint_states.name[j]:
	  positions[i] = joint_states.position[j]
	  velocities[i] = joint_states.velocity[j]
	  effort[i] = joint_states.effort[j]
	  
def shm_omnibase(joint_states):
    for i in range(len(joints)):
      for j in range(len(joint_states.name)):
	if joints[i] == joint_states.name[j]:
	  positions[i] = joint_states.position[j]
	  velocities[i] = joint_states.velocity[j]
	  effort[i] = joint_states.effort[j]

def shm_zlift(joint_states):
    for i in range(len(joints)):
      for j in range(len(joint_states.name)):
	#if joints[i] == joint_states.name[j]:
	if ("zlift_joint" == joint_states.name[j]) and (joints[i] == "torso_lift_joint"):
	  positions[i] = joint_states.position[j]/1000.0
	  velocities[i] = joint_states.velocity[j]
	  effort[i] = joint_states.effort[j]


rospy.init_node("joint_state_publisher")
pub = rospy.Publisher("/joint_states", JointState)


joints = []
positions = []
velocities = []
effort = []

# For the base
joints.append('X')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

joints.append('Y')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

joints.append('yaw')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

#joints.append('zlift_joint')
#positions.append(0.0)
#velocities.append(0.0)
#effort.append(0.0)

joints.append('torso_lift_joint')
positions.append(0.55) # set the default to our usual default height
velocities.append(0.0)
effort.append(0.0)

# For the Right Arm
joints.append('right_arm_j0')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_arm_j1')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_arm_j2')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_arm_j3')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_arm_j4')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_arm_j5')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_arm_j6')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

# For the left arm
joints.append('left_arm_j0')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_arm_j1')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_arm_j2')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_arm_j3')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_arm_j4')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_arm_j5')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_arm_j6')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

# For the head
joints.append('head_j0')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j1')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j2')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j3')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j4')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j5')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j6')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j7')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j7_rt_eyelid_bottom')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j7_lt_eyelid_bottom')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j7_lt_eyelid_top')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j8')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j9')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j10')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('head_j11')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

# For the left hand
joints.append('left_hand_j0')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j1')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j2')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j3')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j4')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j5')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j6')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j7')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j8')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j9')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j10')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('left_hand_j11')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)

# For the right hand
joints.append('right_hand_j0')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j1')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j2')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j3')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j4')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j5')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j6')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j7')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j8')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j9')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j10')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)
joints.append('right_hand_j11')
positions.append(0.0)
velocities.append(0.0)
effort.append(0.0)



header = Header(0,rospy.Time.now(),'0')
pub.publish(JointState(header, joints, positions, velocities, effort))

rospy.Subscriber("/humanoid_state", JointState, shm_humanoid)

rospy.Subscriber("/zlift_state", JointState, shm_zlift)

rospy.Subscriber("/omnibase_state", JointState, shm_omnibase)

try:
    while not rospy.is_shutdown():
	time.sleep(0.01)	
	header = Header(0,rospy.Time.now(),'0')
	pub.publish(JointState(header, joints, positions, velocities, effort))
except (KeyboardInterrupt,EOFError,rospy.ROSInterruptException):
    pass




