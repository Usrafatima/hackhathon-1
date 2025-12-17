---
title: 'Chapter 9: Robot Navigation and SLAM'
metadata:
  chapter_number: 9
  keywords: ['navigation', 'slam', 'localization', 'path planning', 'obstacle avoidance', 'amcl', 'move_base']
---

# Chapter 9: Robot Navigation and SLAM


## Goals


- **To introduce the components of the autonomous navigation stack.** We will break down the complex problem of getting from point A to point B into four distinct sub-problems.
- **To explain the problem of Simultaneous Localization and Mapping (SLAM)** and understand why it is a foundational capability for mobile robots.
- **To describe the concepts of localization, global path planning, and local obstacle avoidance.**

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **List and describe the four main components** of a robot navigation system: Mapping, Localization, Path Planning, and Obstacle Avoidance.
- **Explain what the SLAM problem is** and why it is often called a "chicken and egg" problem.
- **Differentiate between a global planner and a local planner** and describe the role of each.

## Full Explanations of Concepts



### The Challenge of Autonomous Navigation

One of the most fundamental capabilities for any mobile robot is the ability to navigate its environment autonomously. This single goal can be broken down into four essential questions that the robot must continuously answer:

1.  **"What does the world look like?" (Mapping):** Does the robot have a pre-existing map, or does it need to build one from scratch?
2.  **"Where am I?" (Localization):** Given a map, how can the robot determine its own position and orientation (its pose) within that map?
3.  **"Which way should I go?" (Path Planning):** Given its location and a goal location, what is the best route to take?
4.  **"How do I move without hitting things?" (Obstacle Avoidance):** How does the robot follow the planned path while safely avoiding unexpected or moving obstacles (like people)?

The ROS Navigation Stack is a collection of software packages designed to solve these problems in a generic way for any mobile robot.

### SLAM: Simultaneous Localization and Mapping

In many real-world scenarios, a robot is placed in a completely new environment where no map exists. To navigate, it must build the map itself. This is the SLAM problem.

- **The Problem Definition:** SLAM is the process by which a robot builds a map of an unknown environment while simultaneously keeping track of its own location within that map.

- **The "Chicken and Egg" Problem:** SLAM is famously difficult because of its recursive nature.
    - To build an accurate map, you must know the precise location from which each sensor measurement was taken. If you think you've moved one meter but you've actually moved 1.1 meters, your new sensor readings won't line up with your old ones, and the map will become corrupted and distorted.
    - But to determine your precise location, you need an accurate map to compare your current sensor readings against.
    - In short: **You need a good map to localize, and you need good localization to build a map.**

- **The Solution:** SLAM algorithms solve this by treating the map and the robot's pose as a single, large, interconnected system of probabilities. They iteratively update both at the same time. A key part of the solution is **loop closure**.
    - **Loop Closure:** When the robot travels in a loop and returns to a place it has been before, its sensors will recognize the location. This event is a "loop closure." It provides a powerful constraint for the algorithm. The algorithm can then work backward and "correct" the small errors that have accumulated in the map and the robot's trajectory during the loop.

- **Common SLAM Approaches:**
    - **Filter-based SLAM:** Uses statistical filters (like the Extended Kalman Filter) to solve the problem.
    - **Graph-based SLAM (e.g., `gmapping` in ROS):** This is the more modern and common approach. Each robot pose and sensor measurement is treated as a "node" in a giant graph. The relationships between poses (odometry) and between poses and landmarks are the "edges." When a loop is closed, the algorithm optimizes the entire graph to find the most probable map and trajectory.
    - **Visual SLAM (VSLAM):** A category of SLAM that uses cameras as the primary sensor instead of LiDAR.

### Localization: The "Where Am I?" Problem

Once you have a map (either from SLAM or one that was pre-loaded), the problem becomes simpler: just figure out where you are. This is pure localization. The most common algorithm used in the ROS ecosystem is **AMCL (Adaptive Monte Carlo Localization)**.

- **How AMCL Works (Particle Filter):**
    1.  **Initialization (The "Robot Kidnapping Problem"):** When the robot first starts, it has no idea where it is. The AMCL algorithm scatters thousands of random guesses, called **particles**, all over the map. Each particle represents a possible hypothesis for the robot's pose (e.g., "Maybe I'm at x=1, y=5, facing north").
    2.  **Prediction:** The robot drives a short distance, using its odometry (wheel encoders) to estimate its motion. The algorithm moves all the particles according to this estimated motion.
    3.  **Measurement & Scoring:** The robot takes a sensor reading (e.g., a 360-degree laser scan). The algorithm then goes through every single particle and asks: "If the robot were *really* at this particle's location, how well would the laser scan match the map?" Particles in locations where the scan matches the map's walls get a high score (high probability). Particles in locations where the scan goes through a wall in the map get a very low score.
    4.  **Resampling:** A new set of thousands of particles is created. This is done by randomly drawing from the old set, but with a bias: high-scoring particles are much more likely to be chosen again. Low-scoring particles are likely to be eliminated.
    5.  **Convergence:** After just a few cycles of Prediction and Measurement/Resampling, the particles that are inconsistent with reality die out. The surviving particles will naturally cluster together into a small cloud around the robot's true position. The weighted average of this cloud of particles is the robot's estimated pose.

### Path Planning: "Which Way Should I Go?"

Path planning is broken into two distinct components that work together.

#### Global Planner
- **Job:** To find the best path from the robot's starting position to a far-away goal, considering only the static, known map.
- **Analogy:** Using Google Maps to find the best route from your house to the airport *before* you start your car. It considers major roads but doesn't know about current traffic or a person crossing the street.
- **Algorithms:** Typically uses graph-search algorithms like **Dijkstra** or, more commonly, **A*** (A-star), which is a more efficient version of Dijkstra. It operates on a "costmap" derived from the main map, where areas near walls are given a higher "cost" to encourage the planner to find paths that stay in the middle of open spaces.
- **Output:** An array of waypoints that forms a smooth path from start to finish.

#### Local Planner
- **Job:** To follow the global path while avoiding immediate obstacles.
- **Analogy:** You are driving your car, following the Google Maps route. A pedestrian steps into the road. The local planner is the part of your brain that hits the brakes and steers around the pedestrian, before trying to get back onto the main route.
- **Algorithms:** The local planner operates on a smaller, local costmap that is constantly updated with fresh sensor data. Common algorithms include:
    - **Dynamic Window Approach (DWA):** The planner simulates many possible velocity commands (different speeds and turn rates) for a short time into the future, scores them based on how well they follow the global path and how close they get to obstacles, and sends the best-scoring command to the motors.
    - **Timed Elastic Band (TEB):** This planner treats the global path like a flexible elastic band and tries to deform it locally to move it away from obstacles.
- **Output:** The final velocity commands (`geometry_msgs/Twist` on the `/cmd_vel` topic) that are sent to the robot's base controller.

## Step-by-Step Diagram Explanation

### The ROS Navigation Stack (`move_base`)

![ROS Navigation Stack Architecture](https://i.imgur.com/example-diagram.png)
*(Note: Replace with a diagram of the move_base architecture)*

This flowchart shows how the different nodes and components in the ROS navigation stack work together within the main `move_base` node.

1.  **Inputs:** Several key pieces of information feed into the `move_base` node:
    - **`/map`:** A topic providing the static map of the world, usually from a `map_server` node.
    - **`/tf`:** The robot's coordinate frames, which provide its current pose (position and orientation) on the map. This is published by the localization node (e.g., `amcl`).
    - **`/scan` or `/points`:** Live sensor data from a LiDAR or other sensor, used for local obstacle avoidance.
    - **`/move_base_simple/goal`:** The goal pose, sent by the user (often from `rviz`).
2.  **The `move_base` Node:** This is the central action server that coordinates the entire process. It contains the two planners:
    - **Global Planner:** A box inside `move_base`. It takes the map, the robot's current pose, and the goal pose as input. Its output is a green line on the diagram labeled **"Global Path."**
    - **Local Planner:** Another box inside `move_base`. It takes the `Global Path` and the live `/scan` data as input. It constantly works to generate safe motor commands.
3.  **Output:** An arrow leaves the `Local Planner` and the `move_base` box, pointing to a box representing the **"Robot Base Controller."** This arrow is labeled with the topic **`/cmd_vel`** and represents the final velocity commands that make the robot move.

This diagram shows a clear data flow: from high-level goals and maps down to low-level motor commands, with feedback from localization and sensors at every step.

## Lab Instructions



### Lab 9.1: SLAM and Autonomous Navigation in Simulation

**Reasoning:**
This lab is a capstone exercise that brings together everything from the last few chapters. You will perform the complete workflow of a truly autonomous mobile robot: first exploring and mapping an unknown environment, and then using that map to navigate to arbitrary goal locations.

**Instructions:**
1.  **Setup:** Ensure you have the `turtlebot3_simulations` package installed from the previous lab.
2.  **Part 1: SLAM (Building the Map)**
    - **Step A:** In a terminal, launch the Gazebo simulation environment. The TurtleBot3 package provides a world perfect for this: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`.
    - **Step B:** In a second terminal, launch the SLAM node. The TurtleBot3 package provides a pre-configured launch file for this: `roslaunch turtlebot3_slam turtlebot3_slam.launch`.
    - **Observe:** This command will also open `rviz`. You will see the robot and its laser scan. Initially, the map is empty.
    - **Step C:** In a third terminal, launch the keyboard teleoperation node: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`.
    - **Action:** Carefully drive the robot around the entire simulated room using your keyboard. As you drive, you will see the map (walls in black, free space in grey) being built in `rviz` in real-time. Try to cover the whole area and make a "loop closure" by returning to where you started.
    - **Step D:** Once you have a good map, save it. In a new terminal, run the `map_saver` utility. The `-f` argument specifies the output filename.
      ```bash
      rosrun map_server map_saver -f ~/my_map
      ```
    - This will create `my_map.pgm` (the image) and `my_map.yaml` (metadata) in your home directory. You can now close all the terminals.

3.  **Part 2: Autonomous Navigation**
    - **Step A:** Relaunch the Gazebo simulation: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`.
    - **Step B:** In a new terminal, launch the navigation stack. This time, tell it to use the map you just created:
      ```bash
      roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/my_map.yaml
      ```
    - **Observe:** `rviz` will open again. This time it will display your saved map. You will also see a large, diffuse cloud of red arrows (`/particlecloud`) all over the map. This is AMCL trying to guess the robot's position.
    - **Step C (Localization):** The robot needs an initial hint. In the `rviz` toolbar, click the **"2D Pose Estimate"** button. Then, click and drag on the map at the robot's approximate starting location, with the arrow pointing in the direction it's facing. The particle cloud will instantly shrink to that area.
    - **Step D (Navigation):** In the `rviz` toolbar, click the **"2D Nav Goal"** button. Click and drag anywhere on the map to set a destination for the robot.
    - **Observe:** A green line (the global path) will appear. The robot will begin to move on its own, with a red line (the local path) in front of it. It will drive to the goal and stop. You can give it another goal, and it will happily navigate there as well.

**Expected Outcome:**
- Students will have a complete, practical understanding of the entire mobile autonomy pipeline.
- They will have personally witnessed the process of SLAM creating a map from raw sensor data.
- They will have seen the particle filter (AMCL) in action, localizing the robot.
- They will have successfully commanded a robot to navigate to a goal, observing the global and local planners working together. This lab provides a deep, hands-on understanding of how modern mobile robots work.
