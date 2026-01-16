# pedestrian_detector.py

# Purpose:
# This ROS 2 node represents the "Perception layer" of the AEB system.
# It estimates the distance to a pedestrian based on incoming camera data.

# For simplicity and demonstration of the DevOps pipeline:
# In this demo implementation, pedestrian detection is mocked
# using a randomly generated distance value. In a real system, this
# would be replaced by a computer vision or ML-based detector.

# Role in AEB Pipeline:
# Sensor Input -> Perception -> Decision -> Actuation
# This code performs the second stage of "Perception"

# Safety Relevance:
# Pedestrian distance estimation directly affects Time-To-Collision (TTC)
# and then braking decisions. Any error here propagates downstream, making
# this node safety-critical.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class PedestrianDetector(Node):
    """
    PedestrianDetector subscribes to camera input and publishes
    an estimated distance to the nearest pedestrian.
    This abstraction allows perception logic to be tested independently
    of the camera and decision-making modules.
    """

    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('pedestrian_detector')

        # Subscription:
        # In this demo, camera input is mocked using a Float32 message.
        # In a real implementation, this would subscribe to sensor_msgs/Image.
        self.sub = self.create_subscription(
            Float32,                    # Mocked message type
            '/camera/image_raw',        # Camera topic (mocked for demo)
            self.image_cb,              # Callback function
            10                          # QoS queue depth
        )

        # Publisher:
        # Publishes estimated pedestrian distance in meters
        self.pub = self.create_publisher(
            Float32,                    # Message type
            '/pedestrian_distance',     # Output topic
            10                          # QoS queue depth
        )

    def image_cb(self, msg):
        """
        Callback triggered when camera data is received.
        Parameters:
                msg : std_msgs.msg.Float32
            Mocked camera input (not used in this demo).

        Behavior:
        - Generates a mock pedestrian distance (5–15 meters).
        - Publishes the distance for downstream AEB decision logic.
        - Logs the detected distance for traceability.
        """

        # Mock pedestrian distance in meters
        # In a real system, this would come from image processing / ML inference
        distance = random.uniform(5.0, 15.0)

        # Create and populate output message
        out = Float32()
        out.data = distance

        # Publish estimated distance
        self.pub.publish(out)

        # Log detection for debugging and safety traceability
        self.get_logger().info(f'Pedestrian detected at {distance:.2f} meters')


def main():
    """
    Main entry point for the PedestrianDetector node.
    - Initializes ROS 2
    - Starts perception processing loop
    - Ensures clean shutdown
    """

    # Initialize ROS 2 communication
    rclpy.init()

    # Create node instance
    node = PedestrianDetector()

    # Spin node to process callbacks
    rclpy.spin(node)

    # Cleanup resources
    node.destroy_node()
    rclpy.shutdown()
