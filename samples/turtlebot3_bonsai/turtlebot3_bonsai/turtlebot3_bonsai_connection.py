
from typing import Dict
import numpy as np

# ROS
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TurtleBot3BonsaiConnection(Node):
    def __init__(self, node_name):
        # Calls Node.__init__('listener')
        super().__init__(node_name)

        # constants
        self.MAX_LINEAR_VEL_BURGER = 0.22
        self.MAX_ANGULAR_VEL_BURGER = 2.7
        self.MAX_LIDAR_RANGE = 3.5

        # Subscribe
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self._imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self._laser_scan_callback, 10)

        # Publish
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        
        self.cmd_vel_data = Twist()
        self.odom_data = Odometry()
        self.imu_data = Imu()
        self.laser_scan_data = LaserScan()

        self.state = {}

        self.init_params()

    def init_params(self):
        # cmd_vel
        self.state["angular_velocity_x"] = 0.0
        self.state["angular_velocity_y"] = 0.0
        self.state["angular_velocity_z"] = 0.0
        self.state["linear_velocity_x"] = 0.0
        self.state["linear_velocity_y"] = 0.0
        self.state["linear_velocity_y"] = 0.0

        # odometry
        self.state["last_odometry_position_x"] = 0.0
        self.state["last_odometry_position_y"] = 0.0
        self.state["last_odometry_position_z"] = 0.0
        self.state["odometry_position_x"] = 0.0
        self.state["odometry_position_y"] = 0.0
        self.state["odometry_position_z"] = 0.0
        self.state["odometry_orientation_x"] = 0.0
        self.state["odometry_orientation_y"] = 0.0
        self.state["odometry_orientation_z"] = 0.0
        self.state["odometry_orientation_w"] = 0.0
        self.state["twist_angular_velocity_x"] = 0.0
        self.state["twist_angular_velocity_y"] = 0.0
        self.state["twist_angular_velocity_z"] = 0.0
        self.state["twist_linear_velocity_x"] = 0.0
        self.state["twist_linear_velocity_y"] = 0.0
        self.state["twist_linear_velocity_z"] = 0.0

        # imu
        self.state["last_imu_orientation_x"] = 0.0
        self.state["last_imu_orientation_y"] = 0.0
        self.state["last_imu_orientation_z"] = 0.0
        self.state["imu_orientation_w"] = 0.0
        self.state["imu_orientation_x"] = 0.0
        self.state["imu_orientation_y"] = 0.0
        self.state["imu_orientation_z"] = 0.0
        self.state["imu_orientation_w"] = 0.0
        self.state["imu_angular_velocity_x"] = 0.0
        self.state["imu_angular_velocity_y"] = 0.0
        self.state["imu_angular_velocity_z"] = 0.0
        self.state["imu_linear_acceleration_x"] = 0.0
        self.state["imu_linear_acceleration_y"] = 0.0
        self.state["imu_linear_acceleration_z"] = 0.0

        # laser scan
        self.state["lidar_min_angle"] = 0.0
        self.state["lidar_max_angle"] = 0.0
        self.state["lidar_angle_increment"] = 0.0
        self.state["lidar_time_increment"] = 0.0
        self.state["lidar_scan_time"] = 0.0
        self.state["lidar_range_min"] = 0.0
        self.state["lidar_range_max"] = 0.0

        # extrapolated data
        self.state["nearest_scan_range"] = 0.0
        self.state["nearest_scan_radians"] = 0.0
        self.state["last_scan_range"] = 0.0
        self.state["last_scan_radians"] = 0.0

        # config data
        self.state["goal_pose_x"] = 0.0
        self.state["goal_pose_y"] = 0.0
        self.state["sample_range"] = 1

    def get_odom_state_data(self):
        self.state["last_odometry_position_x"] = self.state["odometry_position_x"]
        self.state["last_odometry_position_y"] = self.state["odometry_position_y"]
        self.state["last_odometry_position_z"] = self.state["odometry_position_z"]
        self.state["last_odometry_orientation_x"] = self.state["odometry_orientation_x"]
        self.state["last_odometry_orientation_y"] = self.state["odometry_orientation_x"]
        self.state["last_odometry_orientation_z"] = self.state["odometry_orientation_x"]
        self.state["last_odometry_orientation_w"] = self.state["odometry_orientation_x"]
        self.state["odometry_orientation_x"] = self.odom_data.pose.pose.orientation.x
        self.state["odometry_orientation_y"] = self.odom_data.pose.pose.orientation.y
        self.state["odometry_orientation_z"] = self.odom_data.pose.pose.orientation.z
        self.state["odometry_orientation_w"] = self.odom_data.pose.pose.orientation.w
        self.state["odometry_position_x"] = self.odom_data.pose.pose.position.x
        self.state["odometry_position_y"] = self.odom_data.pose.pose.position.y
        self.state["odometry_position_z"] = self.odom_data.pose.pose.position.z
        self.state["twist_angular_velocity_x"] = self.odom_data.twist.twist.angular.x
        self.state["twist_angular_velocity_y"] = self.odom_data.twist.twist.angular.y
        self.state["twist_angular_velocity_z"] = self.odom_data.twist.twist.angular.z
        self.state["twist_linear_velocity_x"] = self.odom_data.twist.twist.linear.x
        self.state["twist_linear_velocity_y"] = self.odom_data.twist.twist.linear.y
        self.state["twist_linear_velocity_z"] = self.odom_data.twist.twist.linear.z

    def get_imu_state_data(self):
        self.state["last_imu_orientation_x"] = self.state["imu_orientation_x"]
        self.state["last_imu_orientation_y"] = self.state["imu_orientation_y"]
        self.state["last_imu_orientation_z"] = self.state["imu_orientation_z"]
        self.state["last_imu_orientation_w"] = self.state["imu_orientation_w"]
        self.state["imu_orientation_x"] = self.imu_data.orientation.x
        self.state["imu_orientation_y"] = self.imu_data.orientation.y
        self.state["imu_orientation_z"] = self.imu_data.orientation.z
        self.state["imu_orientation_w"] = self.imu_data.orientation.w
        self.state["imu_angular_velocity_x"] = self.imu_data.angular_velocity.x
        self.state["imu_angular_velocity_y"] = self.imu_data.angular_velocity.x
        self.state["imu_angular_velocity_z"] = self.imu_data.angular_velocity.x
        self.state["imu_linear_acceleration_x"] = self.imu_data.linear_acceleration.x
        self.state["imu_linear_acceleration_y"] = self.imu_data.linear_acceleration.y
        self.state["imu_linear_acceleration_z"] = self.imu_data.linear_acceleration.z

    def get_laser_scan_state_data(self):
        self.state["lidar_min_angle"] = self.laser_scan_data.angle_min
        self.state["lidar_max_angle"] = self.laser_scan_data.angle_max
        self.state["lidar_angle_increment"] = self.laser_scan_data.angle_increment
        self.state["lidar_time_increment"] = self.laser_scan_data.time_increment
        self.state["lidar_scan_time"] = self.laser_scan_data.scan_time
        self.state["lidar_range_min"] = self.laser_scan_data.range_min
        self.state["lidar_range_max"] = self.laser_scan_data.range_max

        # Select only the samples in the prescribed field of view
        #   if the laser scan returns data
        if len(self.laser_scan_data.ranges) > 0:
            filtered_data = []
            scan_data = np.array(self.laser_scan_data.ranges)

            if self.state["sample_range"] == 1:
                filtered_data.append(scan_data[0])
            elif self.state["sample_range"] <= 360:
                # ensure index range is type Int since the data comes from a Json Float32 object
                left_range = -(int(self.state["sample_range"]/2) + int(self.state["sample_range"] % 2))
                right_range = int(self.state["sample_range"]/2)
                
                left_lidar_samples = scan_data[left_range:]
                right_lidar_samples = scan_data[:right_range]
                filtered_data.extend(left_lidar_samples + right_lidar_samples)

            # Transform Inf values to the max lidar range
            scan_readings = []

            for i in filtered_data:
                if i == float('Inf'):
                    scan_readings.append(self.MAX_LIDAR_RANGE)
                if i == float('-Inf'):
                    scan_readings.append(self.MAX_LIDAR_RANGE)
                else:
                    scan_readings.append(float(i))
        
            # Save the nearest 
            self.state["last_scan_range"] = self.state["nearest_scan_range"]
            self.state["last_scan_radians"] = self.state["nearest_scan_radians"]
            self.state["nearest_scan_range"] = float(np.min(scan_readings))
            self.state["nearest_scan_radians"] = float((np.amin(scan_readings) - float(self.state["sample_range"]/2)) * float(self.state["lidar_angle_increment"]))

    def _odom_callback(self, data):
        self.odom_data = data

    def _imu_callback(self, data):
        self.imu_data = data

    def _laser_scan_callback(self, data):
        self.laser_scan_data = data