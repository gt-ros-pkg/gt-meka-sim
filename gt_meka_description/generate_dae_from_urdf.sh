#!/bin/bash

## Takes the name of the robot as a single argument (ex. "./generate_dar_from_urdf.sh darci")

# Generate dae from URDF
rosrun collada_urdf urdf_to_collada ./urdf/$1.urdf ./collada/$1.dae 

# If joint axes/anchors have pretty big decmials, round off those (using moveit_ikfast's script).
rosrun gt_meka_description round_collada_numbers.py ./collada/$1.dae ./collada/$1.rounded.dae 3
