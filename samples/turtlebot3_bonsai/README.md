# TURTLEBOT3 BONSAI
This sample provides a Dockerfile and associated code to train reinforecment learning policies for Turtlebot3 using Microsoft Bonsai. 

## Getting Started
This sample assumes that you have access to an Azure Subscription and have set up a Bonsai workspace. See [Microsoft Account Setup](https://docs.microsoft.com/en-us/bonsai/guides/account-setup).

You will need a Linux build environment, azure CLI, and ROS2 Foxy. 

## Build and Run Locally
Clone this repo into the src/ directory of your ROS2 workspace with `--recurse-submodules`. Also set an environement variable with `export TURTLEBOT3_MODEL=burger`

Note: the <world> options are 
* warehouse
* racetrack
* small_house
* bookstore

These worlds refoer to OSS gazebo world models created by AWS Robotics. 

Then, navigate to the samples/ directory.

If you want to run the simulator locally:
* `cd turtlebot3_bonsai`
* `colcon build`
* `source install/setup.bash`
* `ros2 launch turtlebot3_bonsai <world>.launch.py`

If you want to create a local docker container:
* navigate to the samples/ directory 
* `docker image build -f turtlebot3_bonsai/Dockerfile . -t <image name> --build-arg WORLD=<world>

If you want to create a docker image in your Azure Container Registry:
* az login
* az acr login -n <bonsai workspace name>
* az acre build --image <image name> --registry <bonsai workspace name> --file turtlebot3_bonsai/Dockerfile . --build-arg WORLD=<world>

## Setup Cloud Sim with Bonsai
Follow step 2 of [Adding a Training Simulator](https://docs.microsoft.com/en-us/bonsai/guides/add-simulator?tabs=add-cli%2Ctrain-inkling&pivots=sim-platform-other) to associate this docker image with your Bonsai workspace

## Training
See the turtlebot3_bonsai/inkling/ directory for an example inkling file

## Export Brain to Turtlebot3
Once you are satisfied with your Bonsai brain, you can export it at the top right of the training window. 

If you want to run the brain in the simulation loop, export the brain for linux-amd64 and follow the deployment instructions. Then launch the policy_with_sim.alunch.py file. This launch file defaults to using the bookstore world. 

If you want to run the brain on your turtlebot3, export the brain for linux-arm32v7 and follow the deployment instructions. Note that if you have a turtlebot3 running a pi 4, you may need to export the brain for linux-arm64v8. On the robot, run `ros2 launch turtlebot3_bringup robot.launch.py` and `ros2 launch turtlebot3_bonsai policy_without_sim.launch.py`. 

## Other 
If you wish to change the behavior of the simulation itself (such as adding a new sensor to the robot), you may need to edit some or all of the following files:
* config/turtlebot3_sim_interface.json
* turtlebot3_sim_connection.py
* turtlebot3_sim_.py
* <world>.launch.py