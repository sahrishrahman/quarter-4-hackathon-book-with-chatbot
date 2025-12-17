# Lesson 3: Simulating Sensors - LiDAR, Depth Cameras, and IMUs

## Introduction

Realistic sensor simulation is crucial for developing robust robot perception systems. This lesson focuses on simulating three critical sensor types: LiDAR for 3D mapping and navigation, depth cameras for environment understanding, and IMUs (Inertial Measurement Units) for orientation and motion tracking. You will learn how to configure these sensors in simulation environments and process their data to enable robot perception capabilities essential for humanoid robots operating in complex environments.

## Hands-on Exercise

Configure and test different sensor types in simulation:
1.  **LiDAR Simulation**: Set up a 2D or 3D LiDAR sensor in Gazebo or Unity, configure its parameters (range, resolution, field of view), and visualize the point cloud data it generates.
2.  **Depth Camera Setup**: Configure a depth camera in your simulation environment, adjust its resolution and field of view, and process the depth data to identify obstacles and surfaces.
3.  **IMU Integration**: Add an IMU sensor to your robot model to track orientation, angular velocity, and linear acceleration in the simulated environment.
4.  **Sensor Fusion**: Combine data from multiple sensors to improve robot perception (e.g., using LiDAR for mapping and IMU for orientation).
5.  **Obstacle Detection**: Implement basic obstacle detection algorithms using sensor data to enable navigation and collision avoidance.

Sample Gazebo sensor configuration:
```xml
<!-- LiDAR Sensor -->
<sensor name="lidar_sensor" type="ray">
  <pose>0.1 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <topic_name>laser_scan</topic_name>
    <frame_name>lidar_link</frame_name>
  </plugin>
</sensor>

<!-- Depth Camera Sensor -->
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.2 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <topic_name>depth_camera</topic_name>
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

## Summary

Simulating realistic sensors is essential for developing and testing robot perception systems. LiDAR, depth cameras, and IMUs provide complementary information that enables robots to understand their environment, navigate safely, and maintain proper orientation. Proper sensor simulation allows for comprehensive testing of perception algorithms before deployment on real hardware.
