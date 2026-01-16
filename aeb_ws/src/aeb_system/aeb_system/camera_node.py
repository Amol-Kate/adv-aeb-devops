# camera_node.py

# Purpose:
# This ROS 2 node acts as a camera interface adapter for the AEB system.
# It subscribes to raw camera images (e.g., from CARLA simulator or real sensor of the car)
# and republishes them on an internal topic used by downstream perception nodes.

# Role in AEB Pipeline:
# Sensor Input - Perception - Decision - Actuation
# This node represents the first "Sensor Input" stage.

# Vehicle Safety Relevance:
# Camera data integrity is critical for pedestrian detection and
# downstream AEB decision-making. This node does NOT modify image data,
# ensuring sensor data is passed transparently.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    """
    CameraNode subscribes to an external camera topic and republishes
    the image data on an internal, system-defined topic.
    This abstraction allows the AEB system to remain independent of
    the actual camera source (simulator vs real hardware).
    """

    def __init__(self):
        # Initialize the ROS 2 node with a unique name
        super().__init__('camera_node')

        # Subscription: Listens to raw camera images coming from CARLA or a real sensor
        self.sub = self.create_subscription(
            Image,                       # Message type
            '/carla/camera/image_raw',   # Input topic (sensor source)
            self.image_cb,               # Callback function
            10                           # QoS queue depth
        )

        # Publisher: Republishes images on an internal topic for perception modules
        self.pub = self.create_publisher(
            Image,                       # Message type
            '/camera/image_raw',         # Output topic (AEB internal)
            10                           # QoS queue depth
        )

    def image_cb(self, msg):
        """
        Callback function triggered whenever a new camera image is received.
        Parameters:
        msg : sensor_msgs.msg.Image
        The incoming camera image message.

        Behavior:
        - Forwards the image without modification.
        - No processing or filtering is performed here.
        - nee to ensure low latency and data fidelity.
        """

        # Publish the received image to the internal topic
        self.pub.publish(msg)


def main():
    """
    Main entry point for the CameraNode.
    - Initializes the ROS 2 Python client library
    - Creates and spins the CameraNode
    - Need to ensure clean shutdown
    """

    # Initialize ROS 2 communication
    rclpy.init()

    # Create the camera node instance
    node = CameraNode()

    # Keep the node alive to process callbacks
    rclpy.spin(node)

    # Cleanup before shutdown
    node.destroy_node()
    rclpy.shutdown()
