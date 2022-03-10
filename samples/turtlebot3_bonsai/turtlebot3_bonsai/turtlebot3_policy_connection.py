import requests
import json
from typing import Dict
import json
import uuid
import os
import time

from turtlebot3_bonsai.turtlebot3_bonsai_connection import TurtleBot3BonsaiConnection
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import rclpy

NODE_NAME = "turtlebot3_policy_connection"
ROBOT_NAME = "turtlebot"

class PolicyConnection(TurtleBot3BonsaiConnection):
    def __init__(self, policy_url, sim=False, pose_x=0.0, pose_y=0.0, pose_z=0.0):
        super().__init__(NODE_NAME)

        # General variables
        self.url = policy_url
        self.predictPath = "/v2/clients/{clientId}/predict"
        self.headers = {
            "Content-Type": "application/json"
        }
        
        # Set a random UUID for the client.
        # The same client ID will be used for every call
        self.myClientId = str(uuid.uuid4())

        # Build the endpoint reference
        self.endpoint = self.url + self.predictPath.replace("{clientId}", self.myClientId)

        # Initialize node timer
        self.event_timer = self.create_timer(
            0.250,  # unit: s
            self.post_state_data)

        # self.state["goal_pose_x"] = 1     # random number
        # self.state["goal_pose_y"] = -1    # random number

        if sim:
            self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
            while not self.spawn_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('service not available, waiting again...')

            self.sdf = os.path.join(
                get_package_share_directory("turtlebot3_gazebo"), "models",
                "turtlebot3_burger", "model.sdf")

            spawn = SpawnEntity.Request()
            spawn.xml = open(self.sdf, 'r').read()
            spawn.name = ROBOT_NAME
            spawn.robot_namespace = ""
            spawn.initial_pose.position.x = pose_x
            spawn.initial_pose.position.y = pose_y
            spawn.initial_pose.position.z = pose_z

            self.get_logger().info("Sending service request to `/spawn_entity`")
            self.spawn_client.call_async(spawn)

            time.sleep(0.3)

    def post_state_data(self):
        self.get_imu_state_data()
        self.get_odom_state_data()
        self.get_laser_scan_state_data()

        print("---------------------------")
        print("Commanded Linear Velocity: {}".format(self.cmd_vel_data.linear.x))
        print("Commanded Angular Velocity: {}".format(self.cmd_vel_data.angular.z))
        print("Actual Linear Velocity: {}".format(self.state["twist_linear_velocity_x"]))
        print("Actual Angular Velocity: {}".format(self.state["twist_angular_velocity_z"]))
        print("Distance to Nearest Object: {}".format(self.state["nearest_scan_range"]))
        print("Angle to Nearest Object: {}".format(self.state["nearest_scan_radians"]))
        print("---------------------------")

        # Set the request variables
        requestBody = {
        "state": {
            # "wheel_left_joint_position": self.state["wheel_left_joint_position"],
            # "wheel_left_joint_velocity": self.state["wheel_left_joint_velocity"],
            # "wheel_right_joint_position": self.state["wheel_right_joint_position"],
            # "wheel_right_joint_velocity": self.state["wheel_right_joint_velocity"],
            "angular_velocity_x": self.state["angular_velocity_x"],
            "angular_velocity_y": self.state["angular_velocity_y"],
            "angular_velocity_z": self.state["angular_velocity_z"],
            "linear_velocity_x": self.state["linear_velocity_x"],
            "linear_velocity_y": self.state["linear_velocity_y"],
            "linear_velocity_y": self.state["linear_velocity_y"],
            "odometry_position_x": self.state["odometry_position_x"],
            "odometry_position_y": self.state["odometry_position_y"],
            "odometry_position_z": self.state["odometry_position_z"],
            "odometry_orientation_x": self.state["odometry_orientation_x"],
            "odometry_orientation_y": self.state["odometry_orientation_y"],
            "odometry_orientation_z": self.state["odometry_orientation_z"],
            "odometry_orientation_w": self.state["odometry_orientation_w"],
            "last_odometry_position_x": self.state["last_odometry_position_x"],
            "last_odometry_position_y": self.state["last_odometry_position_y"],
            "last_odometry_position_z": self.state["last_odometry_position_z"],
            "last_odometry_orientation_x": self.state["last_odometry_orientation_x"],
            "last_odometry_orientation_y": self.state["last_odometry_orientation_y"],
            "last_odometry_orientation_z": self.state["last_odometry_orientation_z"],
            "last_odometry_orientation_w": self.state["last_odometry_orientation_w"],
            "twist_angular_velocity_x:": self.state["twist_angular_velocity_x"],
            "twist_angular_velocity_y": self.state["twist_angular_velocity_y"],
            "twist_angular_velocity_z": self.state["twist_angular_velocity_z"],
            "twist_linear_velocity_x": self.state["twist_linear_velocity_x"],
            "twist_linear_velocity_y": self.state["twist_linear_velocity_y"],
            "twist_linear_velocity_z": self.state["twist_linear_velocity_z"],
            "imu_orientation_x": self.state["imu_orientation_x"],
            "imu_orientation_y": self.state["imu_orientation_y"],
            "imu_orientation_z": self.state["imu_orientation_z"],
            "imu_orientation_w": self.state["imu_orientation_w"],
            "last_imu_orientation_x": self.state["last_imu_orientation_x"],
            "last_imu_orientation_y": self.state["last_imu_orientation_y"],
            "last_imu_orientation_z": self.state["last_imu_orientation_z"],
            "last_imu_orientation_w": self.state["last_imu_orientation_w"],
            "imu_angular_velocity_x": self.state["imu_angular_velocity_x"],
            "imu_angular_velocity_y": self.state["imu_angular_velocity_y"],
            "imu_angular_velocity_z": self.state["imu_angular_velocity_z"],
            "imu_linear_acceleration_x": self.state["imu_linear_acceleration_x"],
            "imu_linear_acceleration_y": self.state["imu_linear_acceleration_y"],
            "imu_linear_acceleration_z": self.state["imu_linear_acceleration_z"],
            "lidar_min_angle": self.state["lidar_min_angle"],
            "lidar_max_angle": self.state["lidar_max_angle"],
            "lidar_angle_increment": self.state["lidar_angle_increment"],
            "lidar_time_increment": self.state["lidar_time_increment"],
            "lidar_scan_time": self.state["lidar_scan_time"],
            "lidar_range_min": self.state["lidar_range_min"],
            "lidar_range_max": self.state["lidar_range_max"],
            "goal_pose_x": self.state["goal_pose_x"],
            "goal_pose_y": self.state["goal_pose_y"],
            "nearest_scan_range": self.state["nearest_scan_range"],
            "nearest_scan_radians": self.state["nearest_scan_radians"],
            "last_scan_range": self.state["last_scan_range"],
            "last_scan_radians": self.state["last_scan_radians"]
            }
        }

        # Send the POST request
        response = requests.post(
                    self.endpoint,
                    data = json.dumps(requestBody),
                    headers = self.headers
                )

        # Extract the JSON response
        prediction = response.json()

        self.cmd_vel_data.linear.x = prediction['concepts']['PickOne']['action']['input_linear_velocity_x']
        self.cmd_vel_data.angular.z = prediction['concepts']['PickOne']['action']['input_angular_velocity_z']

        # Command Turtlebot3
        self.cmd_vel_pub.publish(self.cmd_vel_data)

    def run(self):
        while True: 
            rclpy.spin_once(self)
            self.post_state_data()
            time.sleep(0.5)
