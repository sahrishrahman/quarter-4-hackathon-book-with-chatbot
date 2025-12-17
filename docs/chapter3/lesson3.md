# Lesson 3: Nav2 - Path Planning for Bipedal Humanoid Movement

## Introduction

Navigation2 (Nav2) is ROS 2's state-of-the-art navigation framework, providing advanced path planning and execution capabilities. This lesson focuses on adapting Nav2 for bipedal humanoid robots, which have unique locomotion requirements compared to wheeled or tracked robots. You will learn how to configure Nav2 for legged robot navigation, including specialized planners that account for bipedal movement constraints, footstep planning, and dynamic balance considerations essential for humanoid robot mobility.

## Hands-on Exercise

Configure Nav2 for humanoid robot navigation:
1.  **Install and Set up Nav2**: Install the Navigation2 stack and familiarize yourself with its components and configuration files.
2.  **Configure Robot Model**: Set up your humanoid robot model with appropriate collision geometry and kinematic constraints for navigation planning.
3.  **Customize Costmaps**: Adjust costmap parameters to account for humanoid-specific navigation challenges like step height limitations and balance constraints.
4.  **Implement Footstep Planning**: Configure or develop footstep planning algorithms that generate safe stepping sequences for bipedal locomotion.
5.  **Test Navigation**: Execute navigation tasks in simulation, observing how the humanoid robot plans and executes paths while maintaining balance and avoiding obstacles.

Sample Nav2 configuration for humanoid navigation:
```yaml
# Costmap configuration for humanoid navigation
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 10.0
  width: 10
  height: 10
  resolution: 0.05
  origin_x: -5.0
  origin_y: -5.0
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
  inflation_layer:
    inflation_radius: 0.8  # Account for humanoid width and balance margin
    cost_scaling_factor: 3.0

global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  static_map: true
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
  inflation_layer:
    inflation_radius: 1.0  # Larger radius for humanoid safety margin

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # DWB Controller parameters for humanoid movement
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: -0.2
      min_vel_y: -0.1
      max_vel_x: 0.5
      max_vel_y: 0.1
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
```

## Summary

Nav2 provides sophisticated path planning capabilities that can be adapted for bipedal humanoid robots with specialized configuration. The framework's flexibility allows for custom planners and controllers that account for the unique locomotion requirements of legged robots. Proper configuration of costmaps, controllers, and safety margins is essential for enabling safe and effective navigation of humanoid robots in complex environments.