# Lesson 3: Understanding URDF and Robot Description

## Introduction

URDF (Unified Robot Description Format) is the standard XML format used in ROS 2 to describe robot models, including their physical structure, kinematic properties, and visual appearance. For humanoid robots, URDF defines the arrangement of links (rigid parts) and joints (connection points), specifying how the robot moves and interacts with its environment. Understanding URDF is crucial for creating accurate robot models that can be used in simulation and real-world applications.

## Hands-on Exercise

Create and manipulate a URDF model for a simple robot:
1.  **Examine a Sample URDF**: Study a provided URDF file that describes a simple robot arm with multiple joints and links, understanding the structure and key elements like `<link>`, `<joint>`, and `<material>`.
2.  **Create Basic URDF**: Write a URDF file that describes a simple wheeled robot with a base, two wheels, and a camera mounted on top. Define the physical properties, visual appearance, and kinematic relationships.
3.  **Visualize in RViz**: Use RViz (ROS visualization tool) to visualize your URDF model and verify that joints move correctly.
4.  **Modify Joint Properties**: Experiment with different joint types (revolute, continuous, prismatic) and observe how they affect the robot's range of motion.
5.  **Add Sensors**: Include sensor definitions in your URDF, such as a camera or IMU, understanding how these are represented in the robot model.

Sample URDF structure:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Wheel Links -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="-0.15 -0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Summary

URDF is fundamental to ROS 2 robotics, providing a standardized way to describe robot geometry, kinematics, and dynamics. Properly defined URDF files enable accurate simulation, visualization, and control of robots. For humanoid robots, URDF becomes particularly important as it captures the complex kinematic chains that enable human-like movement and interaction.
