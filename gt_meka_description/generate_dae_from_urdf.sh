#!/bin/bash

# Generate dae from URDF
rosrun collada_urdf urdf_to_collada ./urdf/darci.urdf ./collada/darci.dae 

# If joint axes/anchors have pretty big decmials, round off those (using moveit_ikfast's script).
rosrun gt_meka_description round_collada_numbers.py ./collada/darci.dae ./collada/darci.rounded.dae 3