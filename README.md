# Turtlebot3 Sample
If you are here for the Turtlebot3 Simulator for Bonsai, please see the [Turtlebot3 Bonsai README](./samples/turtlebot3_bonsai/README.md)

# ROS Connector
A python connector for integrating Bonsai with robotic simulations that use ROS as a message passing framework. Common simulators for robotics:

* Gazebo
* Webots
* CoppeliaSim
* Unity

This connector is built for ROS2 Foxy Fitzroy.

# Getting Started

To use Bonsai, you must provision a Bonsai workspace in an Azure. Your account will need create resources while using Bonsai, so I recommend using a subscription that you have full user access to if possible.

## Provisioning an Azure Account
Please see directions for [setting up a Microsoft account and a Bonsai workspace](https://docs.microsoft.com/en-us/bonsai/guides/account-setup).

## Setting up your Bonsai Workspace
Once you have selected your workspace in the [Bonsai UI](https://preview.bons.ai/), create a new empty brain project.

![](img/emptybrain.png)

We will come back to this later once the the simulator is ready to be connected to Bonsai.

## Setting up your Connector Workspace

A linux workspace or WLS2 is highly reccommended. 

1. Install WLS2 (see WSL Installation at [Setting up ROS in WLS2](https://jack-kawell.com/2020/06/12/ros-wsl2/))
1. Set up GUI forwarding (see Setting Up GUI Forwarding at [Setting up ROS in WLS2](https://jack-kawell.com/2020/06/12/ros-wsl2/))
1. [Install ROS2 ](https://docs.ros.org/en/foxy/Installation.html)
1. [Configure your ROS2 Environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)
1. Create a workspace [Example](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
1. Clone this repo into your workspace's src directory
1. In the src directory, clone and rename the package_template directory

# Customizing the Connector
The package_template directory contains all of the files necessary to set up a ROS enabled simulation. 

## Create a Bonsai Interface
Bonsai requires a JSON filed referred to as an interface in order to interact with the simulator. The interface defines state, action, and configuration data. Each type of data is used differently while training a policy. 

[Sample Interface](./samples/turtlebot3_bonsai/config/turtlebot3_sim_interface.json)

Each simulation needs its own interface.

## Editing the Files
The connector consists of a base, a simulator connection, and a policy connection. 

The base is the main setup of the ROS communication that both the simulator and policy will use to pass telemetry. 

To begin, you will need a ROS2 workspace. Create a new package and create the following folders: <pacakge_name>/config, <pacakge_name>/launch, and <pacakge_name>/<pacakge_name>. See the turtlebot3 sample for reference. 

Copy the python files from src to the <pacakge_name>/<pacakge_name> directory. These files should be edited as follows. 

### bonsai_ros_base.py
* **__init__()**: Topic subscribers and publishers should be initialized here. A common subscription is to /joint_state_data and a common publisher is to /cmd_vel


* **init_params()**: This function needs to initialize all the state variables (defined in the sim config) to an initialize value, most likely 0.0. 

### sim_connection.py
Note that the constructor for the sim_connection class consumes the package where the simulator interface exists and its name. 
* **__init__()**: For the simulator, additional initialization needs to be done to ensure that the simulation can be reset on command. For Gazebo, this is done through a service call. 


* **step()**: The step function should be developed alongside the simulator interface file's action section. Each action variable corresponds to data that Bonsai will pass to the simulation in order to change its state. Each action will need to be parsed and converted to the requisite ROS message type and published to the topic that will apply the action. For example, changes in velocity should be published to the /cmd_vel topic in the form of a Twist() message.   


* **reset()**: At the end of a simulation, the simulator must be reset in order to continue with a new training episode. The simulator needs to be commanded to reset at the start of this function. 

### policy_connection.py
Note that the constructor for the policy_connection class consumes the URL for communicating with the exported Bonsai policy. If the default instructions for exporting and using a policy were followed, this address is http://localhost:5000.
* **post_state_data())**: This function structures the system (robot) state data into a json message that is then sent to the policy via REST request. 

### Launch files
See turtlebot3_bonsai for example launch files. 

## Local (Unmanaged) Training
Policy training instructions are defined in the [Bonsai UI](https://preview.bons.ai/) space. You will need to go to the empty brain project that you created earlier.

The training configuration is defined using Inkling.

See the [Inkling Programming Language Reference](https://docs.microsoft.com/en-us/bonsai/inkling/) for how to plan your policy.

## Cloud (Managed) Training
Once you are satisfied with your local simulation, you can create a docker file to deploy your simulation in the cloud. See the turtlebot3_bonsai sample for reference. 

# Using the Turtlebot3 Sample
The default turtlebot3 sample can be used as-is to train in the cloud, or it can be edited. In your ROS2 workspace, clone the https://github.com/ROBOTIS-GIT/turtlebot3_simulations repo foxy-devel branch.

Note that the sample currently assumes that the turtlebot3 model is **burger**.

## Scaling to the cloud using the included Dockerfile
Follow the instructions for [Adding a Training Simulator](https://docs.microsoft.com/en-us/bonsai/guides/add-simulator?tabs=add-cli%2Ctrain-inkling&pivots=sim-platform-other) using the Dockerfile located in the turtlebot3 sample directory. 

Once the simulator is available in your Bonsai workspace, you can develop an Inkling file to train a policy. A sample inkling file can be found in the turtlebot3_bonsai package's inkling folder. 

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



