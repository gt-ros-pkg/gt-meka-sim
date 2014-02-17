#!/bin/bash

roslaunch ./launch/darci_param_upload.launch &
sleep 6 

gazebo ./darci.world &
sleep 6 

roslaunch ./launch/darci_default_controllers.launch

