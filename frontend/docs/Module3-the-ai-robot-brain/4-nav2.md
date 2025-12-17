---
title: "Nav2: Path Planning for Bipedal Movement"
sidebar_position: 4
tags: [Nav2, Navigation, Path Planning, Behavior Trees, ROS 2]
---

# Chapter 11: Nav2: Path Planning for Bipedal Movement

We have reached the pinnacle of the AI brain's primary function: taking a destination and autonomously navigating to it. With a map and a real-time pose estimate from our perception stack (like Isaac ROS), we can now use the official navigation stack of ROS 2, **Nav2**, to plan and execute movement.

Nav2 is a powerful, production-ready, and highly configurable suite of packages that provides a complete solution for autonomous navigation. While it was originally designed for wheeled robots, its modular architecture allows us to adapt it for the unique challenges of a bipedal humanoid.

## The Core Components of Nav2

Nav2 is not a single node but a coordinated system of servers, each with a specific responsibility. These are orchestrated by a high-level **Behavior Tree**.

1.  **BT Navigator (The Conductor)**: At the top level, Nav2 uses a Behavior Tree (BT) to manage the overall navigation logic. The default BT specifies a sequence like: "Clear Costmaps -> Compute Path to Goal -> Follow Path." This makes the logic explicit and easily customizable without rewriting code.

2.  **Global Planner (The Map Reader)**: The Planner Server is responsible for finding a complete, long-range path from the robot's current position to its final goal. It operates on the static map provided by the SLAM system. A common global planner is "Grid-Based A*", which searches the map for the shortest valid path, avoiding any known, permanent obstacles.

3.  **Local Planner / Controller (The Obstacle Dodger)**: The Controller Server's job is to follow the global path while reacting to immediate, unforeseen obstacles. It looks at a small "local costmap" generated from live sensor data (like a LiDAR scan) and computes safe velocity commands. Popular controllers include TEB (Timed Elastic Band) and DWA (Dynamic Window Approach), which balance path following with obstacle avoidance and kinematic constraints.

## The Bipedal Challenge: From `Twist` to Footsteps

Here we encounter the most significant challenge in using Nav2 for a humanoid. The standard output of Nav2's Controller Server is a `geometry_msgs/msg/Twist` message. This message specifies a desired linear velocity (forward/backward, side-to-side) and angular velocity (turning). For a differential drive (wheeled) robot, this is perfect; the controller can directly map these velocities to wheel speeds.

For a humanoid, a `Twist` message is woefully insufficient. Bipedal locomotion is a complex, dynamic process involving dozens of joints, balance control, and precise footstep placement. You cannot simply tell a humanoid to move "forward at 0.5 m/s."

### The Solution: A Custom Locomotion Controller

To bridge this gap, we must create a translation layer—a custom controller that sits between Nav2 and our robot's joint controllers. This layer's job is to interpret the simple `Twist` command from Nav2 and convert it into a stream of valid commands for a bipedal walking pattern generator.

<!-- ![Figure 11-1: Adapting Nav2 for Humanoid Locomotion](img/nav2_humanoid_adapter.png) -->
*A diagram showing Nav2's output (a Twist message) being sent not to a base controller, but to a custom "Humanoid Locomotion Controller." This controller translates the desired velocity into a series of footstep placements and joint angle goals, which are then sent to the low-level motor controllers.*

This **Humanoid Locomotion Controller** would:
1.  Subscribe to the `/cmd_vel` topic published by Nav2.
2.  If the `Twist` command has a positive `linear.x` value, it initiates a forward walking gait.
3.  If the `Twist` command has a positive `angular.z` value, it modifies the footstep placement to execute a turn.
4.  If the `Twist` command is zero, it brings the robot to a stable stand.
5.  This controller would then publish `JointTrajectory` messages or similar commands to the individual joint controllers.

This modular approach allows us to leverage the power of Nav2's high-level planning without having to modify its core code. We simply create a new "plugin" or node that speaks the language of our robot's unique locomotion system.

## Configuring Nav2

Nav2 is almost entirely configured via a single `nav2_params.yaml` file. This file specifies which plugins to use for each server and sets their parameters.

**Conceptual `nav2_params.yaml` snippet:**
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    # Here, we would specify our custom controller plugin if we wrote one.
    # For now, we use a standard one and subscribe to its /cmd_vel output.
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      # Parameters for the local planner
      max_vel_x: 0.5
      min_vel_x: 0.0
      max_vel_theta: 0.8
      # ... many other parameters

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      # ... other parameters

bt_navigator:
  ros__parameters:
    use_sim_time: True
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    # ... other BT nodes
```
This file tells Nav2 which algorithms to use, how fast to run its control loops, and what the robot's velocity limits are. For our humanoid, we would set conservative velocity limits that match the stable walking speed of our locomotion controller.

---

### Summary

Nav2 provides the final, crucial piece of our robot's autonomy: the ability to navigate. We learned that it is a sophisticated system composed of a Behavior Tree navigator, a global planner, and a local controller. The primary challenge for using Nav2 with a humanoid is its standard `Twist` velocity command output. We have established that the solution is to create a vital translation layer—a custom locomotion controller that converts these simple commands into the complex joint movements required for bipedal walking. By integrating this custom controller, we can successfully leverage the full power of the ROS 2 navigation stack to bring intelligent, autonomous movement to our humanoid.
