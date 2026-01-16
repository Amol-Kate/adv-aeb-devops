# aeb_decision_node.py

# Purpose:
# This ROS 2 node represents the *"Decision & Control Logic" of the
# Autonomous Emergency Braking (AEB) system.

# It computes the Time-To-Collision (TTC) using pedestrian distance
# and vehicle speed, and decides whether emergency braking should
# be triggered.

# Role in AEB Pipeline:
# --------------------
# Sensor -> Perception -> Decision -> Actuation
# This code performs the 3rd phase "Decision"

# Safety Relevance:
# This node directly commands emergency braking.
# Incorrect logic here can cause:
# - False negatives (missed braking to collision)
# - False positives (unnecessary braking  to rear-end risk)

# Therefore, this logic is safety-critical and must be:
# - Deterministic
# - Traceable
# - Verified under simulation and safety gates

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


# TTC threshold in seconds
# If TTC falls below this value, emergency braking is commanded
TTC_THRESHOLD = 1.5  # seconds


class AEBDecisionNode(Node):
    """
    AEBDecisionNode fuses perception and vehicle state to determine
    whether Autonomous Emergency Braking (AEB) should be activated or not.
    """

    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('aeb_decision_node')

        # Internal state variables
        self.speed = 0.0       # Current vehicle speed (m/s)
        self.distance = None  # Distance to detected pedestrian (m)

        # Subscription:
        # Receives estimated pedestrian distance from perception module
        self.sub_dist = self.create_subscription(
            Float32,
            '/pedestrian_distance',
            self.dist_cb,
            10
        )

        # Subscription:
        # Receives current vehicle speed
        self.sub_speed = self.create_subscription(
            Float32,
            '/vehicle/speed',
            self.speed_cb,
            10
        )

        # Publisher:
        # Publishes emergency braking command (True = Brake)
        self.pub = self.create_publisher(
            Bool,
            '/aeb_brake_cmd',
            10
        )

    def speed_cb(self, msg):
        """
        Callback for vehicle speed updates.

        Parameters:
        msg : std_msgs.msg.Float32
        Vehicle speed in meters per second.

        Safety Handling:
        - Speed is clamped to a minimum value to avoid division by zero
          during TTC computation.
        """

        # Prevent divide-by-zero in TTC calculation
        self.speed = max(msg.data, 0.1)

    def dist_cb(self, msg):
        """
        Callback for pedestrian distance updates.

        Parameters:
        msg : std_msgs.msg.Float32
        Estimated distance to the pedestrian in meters.

        Behavior:
        - Computes Time-To-Collision (TTC)
        - Compares TTC against safety threshold
        - Publishes emergency braking decision
        """

        # Update internal distance state
        self.distance = msg.data

        # Compute Time-To-Collision (TTC)
        # TTC = distance / speed
        ttc = self.distance / self.speed

        # Create braking command message
        brake = Bool()
        brake.data = ttc < TTC_THRESHOLD

        # Publish braking decision
        self.pub.publish(brake)

        # Log decision details for traceability and debugging
        self.get_logger().info(
            f'Distance={self.distance:.2f} m, '
            f'Speed={self.speed:.2f} m/s, '
            f'TTC={ttc:.2f} s, '
            f'BRAKE={brake.data}'
        )


def main():
    """
    Main entry point for the AEBDecisionNode.
    - Initializes ROS 2
    - Runs the decision-making loop
    - Ensures deterministic shutdown
    """

    # Initialize ROS 2 communication
    rclpy.init()

    # Create decision node instance
    node = AEBDecisionNode()

    # Spin node to process callbacks
    rclpy.spin(node)

    # Cleanup resources
    node.destroy_node()
    rclpy.shutdown()
