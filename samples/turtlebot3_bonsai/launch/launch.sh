#!bin/bash

source /opt/ros/foxy/setup.bash 
source /ros2_ws/install/setup.sh

if [ "$SIM_WORLD" == "racetrack" ]
then
    ros2 launch turtlebot3_bonsai racetrack.launch.py gui:=false
elif [ "$SIM_WORLD" == "warehouse" ]
then
    ros2 launch turtlebot3_bonsai warehouse.launch.py gui:=false
elif [ "$SIM_WORLD" == "small_house" ]
then
    ros2 launch turtlebot3_bonsai small_house.launch.py gui:=false
elif [ "$SIM_WORLD" == "bookstore" ]
then
    ros2 launch turtlebot3_bonsai bookstore.launch.py gui:=false
else
    ros2 launch turtlebot3_bonsai small_house.launch.py gui:=false
fi