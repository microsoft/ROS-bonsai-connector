# TURTLEBOT3 BONSAI
This sample provides a Dockerfile and associated code to train reinforecment learning policies for Turtlebot3 using Microsoft Bonsai.

# Getting Started
This sample assumes that you have access to an Azure Subscription and have set up a Bonsai workspace. See [Microsoft Bonsai Account Setup](https://docs.microsoft.com/en-us/bonsai/guides/account-setup).

## Building and Running the Simulator Locally
To run the simulator locally, you will need to install Docker on your machine.

In a command line shell (ex: powershell, bash):

* git clone --recurse-submodules git@github.com:microsoft/ROS-bonsai-connector.git
* navigate to the samples/ directory
* `docker image build -f turtlebot3_bonsai/Dockerfile . -t <image name> --build-arg WORLD=<world>`
* copy and edit the env.example file in the config folder with your Bonsai workspace settings
* `docker container run --env-file=turtlebot3_bonsai/config/<env file> <image name>`

Note: the <world> options are
* warehouse
* racetrack
* small_house
* bookstore

The world determines the environment that the turtlebot3 will be launched in. They reference OSS gazebo models created by AWS Robotics. You can learn more about these worlds at

* https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
* https://github.com/aws-robotics/aws-robomaker-racetrack-world
* https://github.com/aws-robotics/aws-robomaker-small-house-world
* https://github.com/aws-robotics/aws-robomaker-bookstore-world

## Building and Running the Simulator in the Cloud
To add the simulator as a managed option in your Bonsai workspace, you will need to install the azure-cli `pip install azure-cli`.

See [How to Install the Azure CLI](https://docs.microsoft.com/en-us/cli/azure/install-azure-cli) if you would prefer to use a method different from pip.

In a command line shell (ex: powershell, bash):

* navigate to the samples/ directory
* `az login`
* `az acr login -n <bonsai workspace name>`
* `az acr build --image <image name> --registry <bonsai azure container registry name> --file turtlebot3_bonsai/Dockerfile . --build-arg WORLD=<world>`
* In the Bonsai UI, go to '+ Add sim' and select 'Other'
* Add your <image name> to the path to the ACR image and name your sim.
* Recommended settings - OS: Linux, Max Instance Count: 25, Cores: 2, Memory: 4GB

You should now be able to link your brain to this simulator. Note that running this sim will significantly speed up training time at the cost of Azure spend.

Note: if you are unsure of what your Azure Container Registry name is, it is most likely the same name as your Bonsai workspace. You can also find it within a resource group in the Azure portal named `bonsai-rg-<workspace name>-<uuid>`.

## Training
See the turtlebot3_bonsai/inkling/ directory for an example inkling file

## Export Brain to Turtlebot3
Once you are satisfied with your Bonsai brain, you can export it at the top right of the training window.

If you want to run the brain in the simulation loop, export the brain for linux-amd64 and follow the deployment instructions. Then launch the policy_with_sim.alunch.py file. This launch file defaults to using the bookstore world.

If you want to run the brain on your turtlebot3, export the brain for linux-arm32v7 and follow the deployment instructions. Note that if you have a turtlebot3 running a pi 4, you may need to export the brain for linux-arm64v8. On the robot, run `ros2 launch turtlebot3_bringup robot.launch.py` and `ros2 launch turtlebot3_bonsai policy_without_sim.launch.py`.

# Advanced Usage
If you are interested in editing the simulation or running it directly on your machine without a container, you will need a Linux build environment, the azure CLI, and ROS2 Foxy. It is highly recommended that you are familiar with developing and building ROS projects before proceeding.

## Build without Docker

If you want to run the simulator locally without docker:
* `mkdir <workspace name> && cd <workspace name>`
* `mkdir src`
* clone this repo into the src folder
* `colcon build`
* `source install/setup.bash`
* `ros2 launch turtlebot3_bonsai <world>.launch.py`

## Editing the Simulator

If you wish to change the behavior of the simulation itself (such as adding a new sensor to the robot), you may need to edit some or all of the following files:
* config/turtlebot3_sim_interface.json
* turtlebot3_sim_connection.py
* turtlebot3_sim_.py
* <world>.launch.py

## Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.opensource.microsoft.com.

When you submit a pull request, a CLA bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., status check, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Trademarks

This project may contain trademarks or logos for projects, products, or services. Authorized use of Microsoft
trademarks or logos is subject to and must follow
[Microsoft's Trademark & Brand Guidelines](https://www.microsoft.com/en-us/legal/intellectualproperty/trademarks/usage/general).
Use of Microsoft trademarks or logos in modified versions of this project must not cause confusion or imply Microsoft sponsorship.
Any use of third-party trademarks or logos are subject to those third-party's policies.
