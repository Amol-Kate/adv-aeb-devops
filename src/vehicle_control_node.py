# vehicle_control_node.py

# Purpose:
# This ROS 2 node represents the **Actuation layer** of the
# Autonomous Emergency Braking (AEB) system.

# It converts a high-level AEB braking decision into a
# low-level brake command for the vehicle.

# Role in AEB Pipeline:
# Sensor -> Perception -> Decision -> Actuation
# This code performs the 4th stage of Actuation

# Safety Relevance:
# This node directly influences vehicle braking.
# Any malfunction here can result in:
# - Failure to brake during an emergency
# - Unexpected braking behavior

# Therefore, this node must be:
# - Deterministic
# - Fail-safe
# - Easily verifiable


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class VehicleControlNode(Node):
    """
    VehicleControlNode subscribes to the AEB braking decision and
    publishes a corresponding brake command to the vehicle interface.
    """

    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('vehicle_control_node')

        # Subscription:
        # Receives high-level AEB braking command
        # True  -> Apply emergency braking
        # False -> No braking requested
        self.sub = self.create_subscription(
            Bool,
            '/aeb_brake_cmd',
            self.brake_cb,
            10
        )

        # Publisher:
        # Publishes normalized brake command to vehicle interface
        # 0.0 -> No brake
        # 1.0 -> Full brake
        self.pub = self.create_publisher(
            Float32,
            '/vehicle/brake',
            10
        )

    def brake_cb(self, msg):
        """
        Callback triggered when an AEB braking decision is received.

        Parameters:
        msg : std_msgs.msg.Bool
            AEB braking decision.

        Behavior:
        - Converts boolean decision into a numeric brake command.
        - Uses a simple binary mapping for demonstration purposes.
        - Publishes the brake command immediately to minimize latency.

        Fail-Safe Considerations
        - True always maps to maximum braking.
        - False maps to zero braking.
        - No intermediate or ambiguous states are allowed.
        """

        # Create brake command message
        brake_cmd = Float32()

        # Map boolean AEB decision to brake intensity
        brake_cmd.data = 1.0 if msg.data else 0.0

        # Publish brake command
        self.pub.publish(brake_cmd)

        # Log actuation for traceability and debugging
        self.get_logger().info(f'Brake applied: {brake_cmd.data}')


def main():
    """
    Main entry point for the VehicleControlNode.
    - Initializes ROS 2
    - Executes actuator command loop
    - Ensures controlled shutdown
    """

    # Initialize ROS 2 communication
    rclpy.init()

    # Create vehicle control node instance
    node = VehicleControlNode()

    # Spin node to process callbacks
    rclpy.spin(node)

    # Cleanup resources
    node.destroy_node()
    rclpy.shutdown()
