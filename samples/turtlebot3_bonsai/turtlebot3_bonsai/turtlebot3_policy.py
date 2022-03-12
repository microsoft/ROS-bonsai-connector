#!/usr/bin/env python3

import rclpy
import sys
from turtlebot3_bonsai.turtlebot3_policy_connection import PolicyConnection

def parse_boolean(string):
    return string == "True"

def main(args=None):
    try:
        # Setup
        rclpy.init(args=args)
        args = rclpy.utilities.remove_ros_args(sys.argv)
        turtlebot3_policy_connection = PolicyConnection("http://localhost:5000", "PickOne", parse_boolean(args[1]))
        turtlebot3_policy_connection.run()
        
    except KeyboardInterrupt:
        print("Closing policy connection due to keyboard interrupt")
    finally:
        # Cleanup
        turtlebot3_policy_connection.destroy_node()
        rclpy.shutdown()