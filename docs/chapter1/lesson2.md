# Lesson 2: Building Your First ROS 2 Nodes and Communication

## Introduction

In traditional programming, "Hello, World!" confirms your development environment works. In ROS 2, your "Hello, World!" is establishing communication between two nodes. This lesson guides you through creating your first ROS 2 nodes that communicate via topics and services, forming the foundation of distributed robot control systems. You will learn to implement publisher-subscriber patterns and service clients, bridging Python agents with ROS controllers using rclpy.

## Hands-on Exercise

Using ROS 2 and rclpy, create a simple robot communication system:
1.  **Create a Publisher Node**: Write a ROS 2 node that publishes messages to a topic (e.g., `/robot_status`) containing information about the robot's current state.
2.  **Create a Subscriber Node**: Write a ROS 2 node that subscribes to the `/robot_status` topic and logs received messages.
3.  **Implement a Service Server**: Create a service server that responds to requests to perform simple actions (e.g., changing robot state).
4.  **Create a Service Client**: Write a client node that calls the service to request actions.
5.  **Test Communication**: Launch both nodes and verify that messages are properly published, subscribed, and services are responding correctly.

Sample code structure:
```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status: Active at {self.get_clock().now()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    robot_publisher = RobotPublisher()
    rclpy.spin(robot_publisher)
    robot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Establishing communication between ROS 2 nodes is the foundational step in creating distributed robot control systems. The publisher-subscriber pattern enables asynchronous communication, while services provide synchronous request-response interactions. Understanding these communication primitives is essential for building complex robotic systems with multiple interconnected components.
