#!/usr/bin/env python3

import rclpy
from policy_connection import SimulatorConnection

def main(args=None):
    try:
        # Setup
        rclpy.init(args=args)
        policy_connection = PolicyConnection("http://localhost:5000")
        policy_connection.register_simulator()
        rclpy.spin(policy_connection)
    except KeyboardInterrupt:
        print("Closing sim connection due to keyboard interrupt")
    finally:
        # Cleanup
        policy_connection.client.session.delete(
            workspace_name=policy_connection.config_client.workspace,
            session_id=policy_connection.registered_session.session_id,
        )
        print("Unregistered simulator.")

        policy_connection.destroy_node()
        rclpy.shutdown()