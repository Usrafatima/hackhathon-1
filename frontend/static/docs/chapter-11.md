---
title: 'Chapter 11: Task and Motion Planning'
metadata:
  chapter_number: 11
  keywords: ['task planning', 'motion planning', 'moveit', 'rrt', 'pddl', 'behavior trees', 'configuration space']
---

# Chapter 11: Task and Motion Planning


## Goals


- **To introduce the hierarchy of planning in robotics,** explaining the crucial difference between high-level task planning and low-level motion planning.
- **To describe popular and effective motion planning algorithms,** focusing on the intuition behind sampling-based planners.
- **To introduce MoveIt!,** the de facto standard framework for motion planning for robotic arms in the ROS ecosystem.

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **Clearly differentiate between task planning (what to do and in what order)** and **motion planning (how to move to do it).**
- **Conceptually explain how a sampling-based motion planner like RRT works** to find a path in a complex space.
- **Describe the role of MoveIt!** in the ROS ecosystem and list its key capabilities.

## Full Explanations of Concepts



### The Hierarchy of Planning

For a robot to perform a complex, multi-step task like "cleaning the table," it needs to plan at two different levels of abstraction.

#### Task Planning (High-Level)
This is the "brain" of the operation. Task planning deals with **symbolic and logical reasoning** to determine the *sequence of discrete actions* needed to achieve a high-level goal. It answers the question, "What should I do, and in what order?"

- **Example Goal:** `"Make a cup of coffee."`
- **Task Plan Output (a sequence of steps):**
    1. `pick_up(empty_mug)`
    2. `place_at(coffee_machine)`
    3. `press_button(start_brew)`
    4. `wait_for(brew_complete)`
    5. `pick_up(full_mug)`
    6. `place_at(table)`
- **Common Approaches:**
    - **Automated Planning:** Uses formal languages like **PDDL (Planning Domain Definition Language)** to define the possible actions a robot can take and their effects on the world. A planner then searches for a sequence of actions that leads from the initial state to the goal state.
    - **Behavior Trees (BTs):** A popular and more flexible alternative. A BT is a tree of nodes that controls the flow of execution. They are more reactive and easier to build and debug than classical planners. A BT for making coffee might have a main "sequence" node that tries to execute "Get Mug," then "Brew Coffee," then "Deliver Mug" in order.

#### Motion Planning (Low-Level)
This is the "nerves and muscles" of the operation. For each discrete step in the task plan, the motion planner must compute a valid, collision-free, and smooth trajectory for the robot's joints. It answers the question, "Given an action, *how* do I move my body to execute it?"

- **Example Goal:** For the action `pick_up(empty_mug)`, the motion planner's goal is to find a path for the arm from its current configuration to a pre-grasp position near the mug.
- **Motion Plan Output:** A detailed **trajectory**, which is a sequence of joint positions, velocities, and accelerations over time. For a 6-joint arm, this might be a list of 6-dimensional vectors for every 10 milliseconds of the movement.

### The Motion Planning Problem

Motion planning for a high-degree-of-freedom robotic arm is incredibly complex. The key concept to understanding it is **Configuration Space**.

- **Configuration Space (C-Space):** A robot's "configuration" is a complete specification of the position of every point on the robot. For a robotic arm, this is simply the set of all its joint angles.
    - A simple 2-joint arm can be described by two angles, (θ₁, θ₂). Its C-Space is a 2D plane.
    - A typical 6-joint industrial arm has a 6-dimensional C-Space.
- **Obstacles in C-Space:** An obstacle in the real world (like a person or a table) has a corresponding representation in C-Space. The C-Space obstacle is the set of *all robot configurations* (all possible combinations of joint angles) that would result in a collision with the real-world obstacle. These C-Space obstacles are high-dimensional and have bizarre, complex shapes that are impossible to compute explicitly.
- **The Problem:** Motion planning is the problem of finding a valid, continuous path for a single point from a `start configuration` to a `goal configuration` within the robot's high-dimensional C-Space, while ensuring that the path **never enters any of the C-Space obstacle regions.**

### Motion Planning Algorithms

Because C-Space is so complex, modern planners have moved away from trying to explicitly define the obstacle regions.

#### Sampling-Based Planners (The Modern Standard)
This family of algorithms cleverly avoids the complexity of C-Space by probing the space with random samples. They work like this: generate a random robot configuration (a random set of joint angles), then use a collision-checker to see if that configuration is valid.

- **PRM (Probabilistic Roadmap):**
    1. **Learning Phase:** Randomly generate thousands of robot configurations. Keep the ones that are collision-free.
    2. Connect each valid configuration to its nearest neighbors to form a graph, or "roadmap," of valid pathways through the C-space.
    3. **Query Phase:** To get from a start to a goal, connect the start and goal to the nearest nodes on the roadmap and search the graph for a path.
    - **Best for:** Static environments where you can afford the initial time to build the roadmap once.

- **RRT (Rapidly-exploring Random Tree):**
    1. Start by creating a tree with a single node: the robot's `start configuration`.
    2. **Loop:**
        a. Pick a random point in the C-Space.
        b. Find the node in your tree that is closest to this random point.
        c. Try to extend the tree from that closest node by a small amount in the direction of the random point.
        d. If the new branch doesn't cause a collision, add it to the tree.
    3. Keep growing the tree until one of its branches reaches the `goal configuration`. The path from the start to the goal along the tree is your solution.
    - **RRT-Connect:** A popular and powerful variant that grows two trees simultaneously—one from the start and one from the goal—and tries to connect them. This is often much faster.
    - **Best for:** Problems where you need to find a single path quickly. This is the workhorse of MoveIt!.

### MoveIt!: The Motion Planning Framework for ROS

**MoveIt!** is a hugely popular open-source software framework that provides a comprehensive suite of tools for motion planning and manipulation in ROS. It ties together all the concepts we've discussed.

- **Key Components:**
    - **Robot Description (`URDF`/`SRDF`):** MoveIt! uses the standard URDF file to get the robot's kinematic structure. It also uses a crucial **SRDF (Semantic Robot Description Format)** file. The SRDF adds semantic information, such as:
        - Defining **planning groups** (e.g., the "arm" consists of these 6 joints, the "gripper" consists of these 2 joints).
        - Defining named poses (e.g., a "home" or "ready" position).
        - Specifying which pairs of links can be ignored for collision checking (e.g., adjacent links).
    - **Planning Scene:** A central component that maintains a 3D representation of the robot's current state and the surrounding world. You can add objects to the planning scene (e.g., a table, the object you want to grasp) so the planner knows to avoid them.
    - **Planners via Plugin:** MoveIt! uses a plugin architecture to interface with planning libraries. The most common is **OMPL (Open Motion Planning Library)**, which provides highly-optimized implementations of RRT, RRT-Connect, PRM, and many other algorithms.
    - **Inverse Kinematics Solvers via Plugin:** Provides plugins for different IK algorithms to find the joint angles for a desired end-effector pose.
    - **Rviz and Gazebo Integration:** Provides powerful GUI plugins for `rviz` that let you visualize the planning scene, drag the robot's goal state around, and see the planned path. It also integrates with Gazebo for executing trajectories on a simulated robot.

- **The `move_group` Node:** This is the heart of MoveIt!. It's a ROS node that brings all these components together into a single action server. A user's application can send a goal (e.g., "move the end-effector to pose X" or "move the arm to the 'home' joint configuration") to the `move_group` node. MoveIt! will then plan a collision-free trajectory and, if successful, execute it on the robot.

## Step-by-Step Diagram Explanation

### The Task and Motion Planning Hierarchy

![Task and Motion Planning Hierarchy](https://i.imgur.com/example-diagram.png)
*(Note: Replace with a diagram of the TAMP hierarchy)*

This flowchart shows the clear separation between the high-level symbolic planner and the low-level motion planner.

1.  **Top Level: Task Planner**
    - A box labeled **"Task Planner (e.g., Behavior Tree)"** receives a high-level command like `"Get me the soda."`
    - It reasons about the world state and breaks the problem down into a logical sequence of actions.
    - Its output is a symbolic plan, shown as a series of connected blocks: `[Navigate to Table]` → `[Grasp Can]` → `[Navigate to User]`.
2.  **The Bridge:** An arrow points down from each action in the task plan to the motion planning layer.
3.  **Bottom Level: Motion Planner**
    - A box labeled **"Motion Planner (MoveIt!)"** receives one symbolic action at a time.
    - The diagram shows the `[Grasp Can]` action being processed. This symbolic action is translated into a concrete goal for MoveIt!, such as `"Move end-effector to pose [x,y,z,qx,qy,qz,qw]"`.
    - MoveIt! operates within its **"Planning Scene,"** a sub-diagram showing the robot arm and a representation of the table and can (the collision environment).
    - MoveIt! uses a planner like RRT to find a valid path.
    - Its output is a **"Joint Trajectory"**—a detailed, time-stamped series of motor commands—which is sent to the low-level robot controllers.

## Lab Instructions



### Lab 11.1: Interactive Motion Planning with MoveIt! and Rviz

**Reasoning:**
This lab makes the abstract concept of motion planning tangible and visual. By directly interacting with the MoveIt! plugin in Rviz, you can see the planner at work, discover collision-free paths, and understand how the robot's environment constraints its motion.

**Instructions:**
1.  **Setup:** Install ROS and MoveIt!. For a pre-configured example, the Panda robot from Franka Emika is excellent.
    ```bash
    sudo apt-get install ros-<distro>-panda-moveit-config
    ```
    (Replace `<distro>` with your ROS version, e.g., `noetic`).
2.  **Launch the Demo:** The MoveIt! config package for the Panda arm includes a demo launch file that starts Rviz with all the necessary plugins.
    ```bash
    roslaunch panda_moveit_config demo.launch
    ```
3.  **Explore the Rviz Environment:**
    - A window will open showing the Panda robot arm.
    - On the left, in the "Displays" panel, you will see a "MotionPlanning" display type.
    - You will see two versions of the robot: one opaque (the current `start state`) and one semi-transparent green (the `goal state`).
    - There is a blue ball and arrow marker attached to the gripper of the green robot. This is the **interactive marker**.
4.  **Plan a Simple Motion:**
    - **Action:** Click and drag the interactive marker to a new position and orientation in space. The green goal robot will follow your movements.
    - **Action:** In the "MotionPlanning" panel on the left, find the "Planning" tab. Click the **"Plan"** button.
    - **Observe:** The planner (likely RRT-Connect by default) will search for a valid, collision-free path. If successful, you will see the planned trajectory as an animation or a "trail" showing the arm moving from its start state to the goal state.
    - **Action:** Click "Plan" again. You may see a slightly different path, illustrating the random nature of sampling-based planners.
5.  **Introduce a Collision Object:**
    - **Action:** Go to the "Scene Objects" tab in the MotionPlanning panel. Click the **"Add"** button to add a simple primitive, like a Box, into the planning scene. Use the controls to position this box directly in the path of the motion you planned before.
    - **Action:** Go back to the "Planning" tab and click "Plan" again with the same goal.
    - **Observe:** The planner will now automatically find a new path that skillfully maneuvers the arm *around* the box you added. It respects the collision object. If no such path exists, the planner will report failure.
6.  **Plan and Execute (in simulation):**
    - The `demo.launch` file also starts a fake controller node.
    - After finding a successful plan, you can click the **"Execute"** button.
    - **Observe:** The opaque "start state" robot will now move and follow the planned trajectory until it matches the green "goal state" robot.

**Expected Outcome:**
- Students will have a clear, visual understanding of the motion planning problem. They will see how a tool like MoveIt! can find complex, collision-free paths for a high-degree-of-freedom robot arm, and how it can adapt its plans to avoid obstacles in the environment. This lab demystifies motion planning and shows its power.
