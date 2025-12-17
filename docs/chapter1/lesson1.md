# Lesson 1: Introduction to ROS 2 - The Robotic Nervous System

## Introduction

This lesson introduces the fundamental concepts of ROS 2 (Robot Operating System 2), the middleware that serves as the nervous system for modern robotics. ROS 2 provides the infrastructure for robot control, enabling communication between different robot components through Nodes, Topics, and Services. You will learn how ROS 2 facilitates distributed robot control and enables Python agents to bridge with ROS controllers using rclpy. Additionally, you'll understand the importance of URDF (Unified Robot Description Format) for defining humanoid robot structures.

## Hands-on Exercise

For this exercise, we'll explore the core concepts of ROS 2 by simulating a simple robot communication system:
1. **Understanding Nodes**: Create two Python scripts that represent different robot nodes - one for sensor data processing and one for motor control. These nodes will run independently but communicate through ROS 2.
2. **Setting up Topics**: Implement a publisher-subscriber pattern where the sensor node publishes data (e.g., distance measurements) to a topic called `/sensor_data`, and the motor control node subscribes to this topic to make decisions.
3. **Creating Services**: Implement a service that allows one node to request specific actions from another (e.g., a "move_arm" service).
4. **Using rclpy**: Write Python code using the rclpy library to create these nodes and establish communication between them.
5. **URDF Exploration**: Examine a sample URDF file that describes a simple robot arm structure, understanding how joints, links, and physical properties are defined.

Sample rclpy code structure:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = self.create_publisher(String, 'sensor_data', 10)

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(
            String, 'sensor_data', self.listener_callback, 10)
```

## Summary

ROS 2 serves as the nervous system for modern robotics, enabling complex robot systems to communicate and coordinate efficiently. Through Nodes, Topics, and Services, ROS 2 allows for modular robot architectures where different components can be developed independently. The rclpy library bridges Python agents with ROS controllers, while URDF provides a standardized way to describe robot geometry and kinematics, especially crucial for humanoid robots.
