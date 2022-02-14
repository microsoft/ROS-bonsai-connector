#!/usr/bin/env python3

import rclpy
from sim_connection import SimulatorConnection

def main(args=None):
    try:
        # Setup
        rclpy.init(args=args)
        sim_connection = SimulatorConnection("interface_package", "interface_name")
        sim_connection.register_simulator()
        rclpy.spin(sim_connection)
    except KeyboardInterrupt:
        print("Closing sim connection due to keyboard interrupt")
    finally:
        # Cleanup
        sim_connection.client.session.delete(
            workspace_name=sim_connection.config_client.workspace,
            session_id=sim_connection.registered_session.session_id,
        )
        print("Unregistered simulator.")

        sim_connection.destroy_node()
        rclpy.shutdown()