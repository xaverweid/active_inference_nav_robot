#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
import numpy as np


class StartingPosePublisher(Node):
    def __init__(self, x: float, y: float, yaw: float):
        super().__init__('starting_pose_publisher')
        
        # Use TRANSIENT_LOCAL durability so late subscribers can still receive the message
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.publisher = self.create_publisher(
            Point,
            'starting_pose',
            qos_profile
        )
        
        # Create numpy array with the pose
        pose_array = np.array([x, y, yaw])
        
        # Publish using Point message (x, y, z for x, y, yaw)
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(yaw)  # z field stores yaw
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published starting pose as numpy array: {pose_array}'
        )


def main():
    import sys
    
    rclpy.init(args=sys.argv)
    
    # Get values from command line arguments
    if len(sys.argv) > 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
    else:
        x = 0.0
        y = 0.0
        yaw = 0.0
    
    node = StartingPosePublisher(x, y, yaw)
    
    try:
        node.get_logger().info("Publisher running. Press Ctrl+C to stop.")
        rclpy.spin(node)  # Keep alive indefinitely
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
