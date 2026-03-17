#!/usr/bin/env python3
"""
Simple test publisher for SceneGraphObjectViewArray message.
This script publishes test object messages to /scene_graph/object_descriptions topic.
"""

import rclpy
from rclpy.node import Node
from semantic_mapping_interfaces.msg import SceneGraphObjectViewArray, SceneGraphObjectView
from geometry_msgs.msg import Pose, Vector3
import math


class TestObjectPublisher(Node):
    def __init__(self):
        super().__init__('test_object_publisher')
        self.publisher = self.create_publisher(
            SceneGraphObjectViewArray,
            '/scene_graph/object_descriptions',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_test_message)
        self.get_logger().info('Test publisher started on /scene_graph/object_descriptions')

    def publish_test_message(self):
        msg = SceneGraphObjectViewArray()
        
        # Create a test object
        obj = SceneGraphObjectView()
        obj.id = 10000
        obj.name = "Test Object"
        obj.desc = "谁谁谁"
        
        # Set position
        obj.pose.position.x = 1.0
        obj.pose.position.y = 0.0
        obj.pose.position.z = 0.5
        
        # Set orientation (quaternion) - 45 degrees around Z axis
        yaw = math.radians(45)
        obj.pose.orientation.x = 0.0
        obj.pose.orientation.y = 0.0
        obj.pose.orientation.z = math.sin(yaw / 2)
        obj.pose.orientation.w = math.cos(yaw / 2)
        
        # Set dimensions (size)
        obj.dimensions.x = 1.0  # length
        obj.dimensions.y = 0.5  # width
        obj.dimensions.z = 0.8  # height
        
        msg.objects.append(obj)
        
        self.publisher.publish(msg)
        self.get_logger().info('Published test message')


def main(args=None):
    rclpy.init(args=args)
    node = TestObjectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
