#!/usr/bin/env python3

import rclpy
import sys
from turtlebot3_bonsai.turtlebot3_sim_connection import SimulatorConnection

def main(args=None):
    try:
        # Setup
        rclpy.init(args=args)

        # Get turtlebot spawn position from args
        args = rclpy.utilities.remove_ros_args(sys.argv)
        turtlebot3_sim_connection = SimulatorConnection("turtlebot3_bonsai",
                                                        "turtlebot3_sim_interface.json",
                                                        float(args[1]),
                                                        float(args[2]),
                                                        float(args[3]),
                                                        args[4])
        turtlebot3_sim_connection.register_simulator()
        turtlebot3_sim_connection.run()

    except KeyboardInterrupt:
        print("Closing sim connection due to keyboard interrupt")
    finally:
        # Cleanup
        turtlebot3_sim_connection.client.session.delete(
            workspace_name=turtlebot3_sim_connection.config_client.workspace,
            session_id=turtlebot3_sim_connection.registered_session.session_id,
        )
        print("Unregistered simulator.")

        turtlebot3_sim_connection.destroy_node()
        rclpy.shutdown()