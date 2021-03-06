# TURTLEBOT3 BONSAI
This sample provides a Dockerfile and associated code to train AI via reinforcement learning for Turtlebot3 using Microsoft Bonsai. An AI trained using reinforcement learning is called a **policy** and is referred to by Bonsai as a **Brain**.

There following diagram lays out the development flow for this sample project.

```mermaid
flowchart TD
    A[Set Up Project]--Slower & Inexpensive-->B[Set Up Unmanaged Simulator]
    A--Faster & More Expensive-->C[Set Up Managed Simulator]


    C-->D1[Define Bonsai Brain]
    B-->D1[Define Bonsai Brain]
    D1-->D2[Train Bonsai Brain]
    D2-->D3[Deploy Bonsai Brain]
    D3-->D1

```

Training the sample Brain provided using an unmanaged simulator can take 48+ hours. It uses your local machine to produce data for training, so this will depend on your hardware. Training the same Brain on a managed simulator (using cloud compute) will take approximately 18 hours using the suggested compute settings. This time can be improved by using more powerful cloud compute, but note that this will result in higher Azure charges.

1. **Set Up Project**: [Requirements](#requirements)
2. **Set Up Unmanaged Simulator**: [Building and Running the Training Simulator Locally](#building-and-running-the-training-simulator-locally)
3. **Set Up Managed Simulator**: [Building and Running the Training Simulator in the Cloud](#building-and-running-the-training-simulator-in-the-cloud)
4. **Define Bonsai Brain**: [Training](#training)
5. **Train Bonsai Brain**: [Training](#training)
6. **Deploy Bonsai Brain To Simulation**: [Export Brain and Test With The Simulation](#export-brain-and-test-with-the-simulation)

If you are interested in deploying the Brain to a physical Turtlebot3, see [Export Brain to Turtlebot3](#export-brain-to-turtlebot3).

# Getting Started

## Requirements
* An Azure Subscription with owner permission.
* A Bonsai workspace. See [Microsoft Bonsai Account Setup](https://docs.microsoft.com/en-us/bonsai/guides/account-setup).
* [Docker](https://docs.docker.com/get-docker/)

Once you have completed the above, identify the name of the Azure Container Registry (ACR) associated with your Bonsai workspace.

Note: if you are unsure of what your ACR name is, it is most likely the same name as your Bonsai workspace, but without special characters. You can also find it within a resource group in the Azure portal named `bonsai-rg-<workspace name>-<uuid>`.

Then clone the repository with:

    git clone --recurse-submodules https://github.com/microsoft/ROS-bonsai-connector.git

## Building and Running the Training Simulator Locally

1. In a command line shell (ex: PowerShell, bash), navigate to the `samples/` directory with `cd ROS-bonsai-connector\samples\` and run the following command:

    `docker image build -f turtlebot3_bonsai/Dockerfile . -t <image name> --build-arg MODE=train --build-arg WORLD=<world>`

    Note: the \<world\> options are
    * warehouse
    * racetrack
    * small_house
    * bookstore

2. Create an `env.train` file in `turtlebot3_bonsai/config/` directory of the project, copying the contents of the **env.train_example** file.
3. In the `env.train` file, provide a Bonsai workspace ID and an access key. See [Bonasi document - Get your workspace access key](https://docs.microsoft.com/en-us/bonsai/cookbook/get-access-key)
4. In a command line shell, run the command:

    `docker container run --env-file=turtlebot3_bonsai/config/env.train <image name>`

    The world determines the environment that the turtlebot3 will be launched in. They reference OSS gazebo models created by AWS Robotics. You can learn more about these worlds at

    * https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
    * https://github.com/aws-robotics/aws-robomaker-racetrack-world
    * https://github.com/aws-robotics/aws-robomaker-small-house-world
    * https://github.com/aws-robotics/aws-robomaker-bookstore-world

## Building and Running the Training Simulator in the Cloud
To add the simulator as a managed option in your Bonsai workspace, you will need to install the azure-cli `pip install azure-cli`.

See [How to Install the Azure CLI](https://docs.microsoft.com/en-us/cli/azure/install-azure-cli) if you would prefer to use a method different from pip.

In a command line shell (ex: powershell, bash):

* navigate to the samples/ directory
* `az login`
* `az account set -s <subscription name or ID>`
* `az acr login -n <bonsai workspace name>`
* `az acr build --image <image name> --registry <bonsai azure container registry name> --file turtlebot3_bonsai/Dockerfile .  --build-arg MODE=train --build-arg WORLD=<world>`

    If this command fails, try adding `--resource-group bonsai-rg-<workspace name>-<uuid>`. You can find this resource group within your Azure subscription - it was created along with your workspace. Otherwise, contact your subscription administrator to verify that you have adequate permissions.

* In the Bonsai UI, go to '+ Add sim' and select 'Other'
* Add your \<image name\> to the path to the ACR image and name your sim.
* Recommended settings - OS: Linux, Max Instance Count: 25, Cores: 2, Memory: 4GB

You should now be able to link your brain to this simulator. **Note that running this sim will significantly speed up training time at the cost of Azure spend.**

## Training
Create a new empty Brain. See the turtlebot3_bonsai/inkling/ directory for an example inkling file. Copy and paste these contents into the **Teach** workspace of your new empty Brain.

Once you are satisfied with your Bonsai brain, you can export it at the top right of the training window.

## Export Brain and Test With The Simulation

![Alt Text](./images/policy_in_the_loop.gif)

If you want to run the brain in the simulation loop:

1. Select the train tab of your brain. Click export the brain at the top right of the browser and select Linux x64 as your processor architecture.
2. Once the brain is ready under Exported Brains in the lower left portion of the browser, click the dots next to the brain and select View Deployment Instructions. Follow these instructions.
3. In a command line shell (ex: PowerShell, bash), navigate to the `samples/` directory and run the following command:

    `docker image build -f turtlebot3_bonsai/Dockerfile . -t <image name> --build-arg MODE=test --build-arg WORLD=<world>`

    Note: the \<world\> options are
    * warehouse
    * racetrack
    * small_house
    * bookstore

4. Create an `env.test` file in `turtlebot3_bonsai/config/` directory of the project, copying the contents of the **env.test_example** file.
5. In the `env.test` file, provide the name of the Bonsai policy. If you used the sample inkling, the name is policy name is **AvoidObstacles**. If you are on Windows, set DISPLAY=YOUR_LOCAL_IP:0.0. If you are on Linux, leave DISPLAY blank.

   On windows, you can find your IP address by opening a command prompt and typing `ipconfig`.

6. If you are using Windows, you will need to install an X-server such as [VcXsrv](https://sourceforge.net/projects/vcxsrv/) to forward the simulation gui to. If using VcXsrv, run XLaunch from the start menu with all defaults plus *Disable access control*.
7. If you are using Windows, open Powershell and run the command:

    `docker container run --env-file=turtlebot3_bonsai/config/env.test --net=host <image name>`

   If you are using Linux, open a terminal and run the command:

    `xhost +local:docker && docker container run --env-file=turtlebot3_bonsai/config/env.test --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --net=host <image name>`

    Note that this is untested for MacOS and WSL.

## Export Brain to Turtlebot3
If you want to run the brain on your turtlebot3, export the brain for linux-arm32v7 and follow the deployment instructions. Note that if you have a turtlebot3 running a pi 4, you may need to export the brain for linux-arm64v8. On the robot, run `ros2 launch turtlebot3_bringup robot.launch.py` and `ros2 launch turtlebot3_bonsai policy_without_sim.launch.py`.

To see an example application using the Brain, check out this integration with Custom Vision: [Turtlebot3-Photo-Collection](https://github.com/microsoft/Turtlebot3-Photo-Collection).
# Advanced Usage
If you are interested in editing the simulation or running it directly on your machine without a container, you will need a Linux build environment, the azure CLI, and ROS2 Foxy. It is highly recommended that you are familiar with developing and building ROS projects before proceeding.

## Build without Docker

If you want to run the simulator locally without docker:
* `mkdir <workspace name> && cd <workspace name>`
* `mkdir src`
* clone this repo into the src folder
* `colcon build`
* `source install/setup.bash`
* `ros2 launch turtlebot3_bonsai <world>_test.launch.py`

## Editing the Simulator

If you wish to change the behavior of the simulation itself (such as adding a new sensor to the robot), you may need to edit some or all of the following files:
* config/turtlebot3_sim_interface.json
* turtlebot3_sim_connection.py
* turtlebot3_sim_.py
* \<world\>_\<test/train\>.launch.py

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
