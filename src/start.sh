#!/bin/bash

source /home/mars/rosenv/bin/activate
source /opt/ros/noetic/setup.bash

#todo remove this?
#source /home/mars/catkin_ws/devel/setup.bash

roslaunch crawler_controller app.launch 
