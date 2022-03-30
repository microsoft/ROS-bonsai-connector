import requests
import json
import json
import uuid

# ROS
import rclpy

NODE_NAME = "turtlebot3_policy_connection"

class PolicyConnection(BonsasiROSBase):
    def __init__(self):
        super().__init__(NODE_NAME)

        # General variables
        self.url = "http://localhost:5000"
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
            self.command_with_policy)

    def command_with_policy(self):

        # Set the request variables
        requestBody = {
        "state": {
            # fill out json to forward state data to the policy
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

        # Parse action data from predictions
        # Example:
        #   Note that the <concept name> and <action name> are defined in your Inkling file
        # self.action = prediction['concepts']['<concept name>']['action']['<action name>']

        # once the action is parsed, convert it to the required ROS message type and publish to its topic (eg: /cmd_vel)