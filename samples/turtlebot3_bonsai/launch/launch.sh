#!bin/bash

source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.sh

if [ "$SIM_MODE" == "train" ]
then
    ros2 launch turtlebot3_bonsai "$SIM_WORLD"_"$SIM_MODE".launch.py gui:=false
elif [ "$SIM_MODE" == "test" ]
then
    source /usr/share/gazebo-11/setup.sh
    ros2 launch turtlebot3_bonsai "$SIM_WORLD"_"$SIM_MODE".launch.py gui:=true
fi