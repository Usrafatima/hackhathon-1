---
title: 'Chapter 16: Capstone - Voice-Interactive Mobile Manipulator'
metadata:
  chapter_number: 16
  keywords: ['capstone project', 'robotics system', 'integration', 'mobile manipulation', 'voice control']
---

# Chapter 16: Capstone Project - Building a Voice-Interactive Mobile Manipulator


## Goals


- **To integrate the knowledge from all previous chapters** into a single, functional, and impressive robotics project.
- **To provide a step-by-step case study** of how different modules (perception, planning, control, HRI) are designed and combined to create a complete autonomous system.
- **To serve as a blueprint and inspiration** for students to design and execute their own complex robotics projects.

## Learning Outcomes


Upon completing this chapter and the associated project, you will be able to:

- **Design a complete system architecture diagram** for a complex robotics application, identifying all necessary nodes and communication pathways.
- **Trace the flow of information and control** in a robotic system, from a high-level human command down to low-level motor actions.
- **Identify the specific ROS nodes, topics, and actions** required to implement a complex mobile manipulation task like "fetch and deliver."

## Full Explanations of Concepts



### The Project Goal: "Fetch the Red Ball"

We will culminate our learning by building a system that accomplishes a single, clear mission:

> **A human operator will ask a simulated mobile manipulator to "fetch the red ball." The robot must then autonomously locate the ball in the environment, navigate to it, pick it up, and return to the operator.**

This single task is a classic robotics challenge because it elegantly forces the integration of almost every topic we have covered in this book:
- **Human-Robot Interaction:** The task is initiated by a natural language voice command.
- **Computer Vision:** The robot must visually search the environment to find the target object.
- **Navigation:** The robot must safely navigate from its starting point to the object's location.
- **Motion Planning & Manipulation:** The robot must plan and execute a complex arm trajectory to grasp the object.
- **State Management:** A high-level process must manage the overall task, sequencing through the various stages (navigating, picking, returning) in the correct order.

### System Architecture Design

The foundation of any complex ROS project is a clear system architecture. We will design our system as a collection of specialized, communicating nodes.

#### HRI Nodes:
- **`/speech_recognizer`:** A node that uses a library like `pocketsphinx` or a cloud service to convert microphone audio into text. It will publish this text to the `/recognized_speech` topic.
- **`/voice_commander`:** The "project manager" node. It subscribes to `/recognized_speech`. When it hears the target command, it orchestrates the entire rest of the process by calling other nodes and action servers.

#### Perception Nodes:
- **`/camera_driver`:** A standard node that publishes `sensor_msgs/Image` from the robot's camera.
- **`/object_detector`:** A custom node that subscribes to the camera images. It will use computer vision techniques (e.g., color thresholding for simplicity, or a YOLO model for more robustness) to find the red ball. Once found, it will publish the ball's 3D position to a `/ball_location` topic.

#### Navigation Nodes:
- **`/move_base`:** The standard ROS Navigation Stack action server. The `/voice_commander` will act as a client to this server, sending it navigation goals (first to the ball, then back to the start). `move_base` handles the entire process of global and local planning to move the robot's base.

#### Manipulation Nodes:
- **`/move_group`:** The standard MoveIt! action server. The `/voice_commander` will also act as a client to this server. It will send a sequence of goals to `/move_group` to execute the multi-stage picking motion.

#### State Management: The Conductor
The `/voice_commander` node is the most critical piece of custom logic. It must act as a **state machine** or, even better, a **Behavior Tree** to manage the overall task flow. A simplified state machine would look like this:
- `IDLE`: Listen for the "fetch" command.
- `SEARCHING`: Command received. Look for the `/ball_location` topic.
- `NAVIGATING_TO_BALL`: Ball location received. Send goal to `move_base`. Wait for result.
- `MANIPULATING`: Navigation succeeded. Send sequence of goals to `move_group`. Wait for result.
- `RETURNING`: Manipulation succeeded. Send "return to start" goal to `move_base`. Wait for result.
- `TASK_COMPLETE`: Return to `IDLE`.

### The "Data Journey": Tracing the Execution Flow

Let's trace the complete flow of information and action from start to finish:

1.  **Human → Robot (HRI):** The human speaks the command, "Robot, fetch the ball."
2.  **Speech → Text (Perception):** The `/speech_recognizer` node captures this audio, processes it, and publishes the string `"FETCH THE BALL"` to the `/recognized_speech` topic.
3.  **Text → Intent (Task Planning):** The `/voice_commander` node is subscribed to this topic, receives the string, and understands the `FETCH` intent. It now enters its `SEARCHING` state.
4.  **Image → Location (Perception):** The `/object_detector` node continuously processes images from the camera. It finds a red blob, calculates its 3D position relative to the robot, and publishes this `PoseStamped` message to the `/ball_location` topic.
5.  **Location → Navigation Goal (Task Planning):** The `/voice_commander` receives the `/ball_location` pose. It now knows where to go. It transitions to the `NAVIGATING_TO_BALL` state and sends this pose as a goal to the `/move_base` action server.
6.  **Nav Goal → Motor Commands (Navigation):** `move_base` takes over. It plans a global path, and the local planner begins issuing `/cmd_vel` commands to the robot's base. The robot drives across the room. When it arrives at the destination, `move_base` reports `SUCCESS`.
7.  **Nav Success → Arm Goal (Task Planning):** The `/voice_commander` receives the success signal from `move_base`. It transitions to the `MANIPULATING` state. It now sends a series of goals to the `/move_group` action server (e.g., "move to pre-grasp," "open gripper," "move to grasp," "close gripper," "lift").
8.  **Arm Goal → Joint Commands (Manipulation):** `move_it` takes over for each goal. It plans collision-free trajectories for the arm and executes them by sending commands to the arm's joint controllers. When the entire sequence is done, it reports `SUCCESS`.
9.  **Grasp Success → Return Goal (Task Planning):** The `/voice_commander` receives the success signal from `move_it`. It transitions to the `RETURNING` state and sends a pre-defined "home" position as a new goal to `/move_base`.
10. **Return Goal → Mission Complete:** `move_base` drives the robot back to its starting point. When it succeeds, the `/voice_commander` announces "Task Complete" and transitions back to the `IDLE` state, ready for a new command.

## Step-by-Step Diagram Explanation

### System Architecture for the "Fetch" Task

![Fetch Task System Architecture](https://i.imgur.com/example-diagram.png)
*(Note: Replace with a detailed ROS computation graph for this system)*

This diagram provides a blueprint of the entire system, showing the nodes as ovals and the communication channels (topics/actions) as arrows.

- **Central Hub:** The `/voice_commander` node sits at the center, acting as the primary client and task manager.
- **HRI Loop:** A `Human` icon provides input to `/speech_recognizer`, which publishes a `String` on `/recognized_speech` to `/voice_commander`.
- **Perception Loop:** The `/camera` publishes `Image` data to `/object_detector`, which in turn publishes `PoseStamped` data on `/ball_location` to `/voice_commander`.
- **Action Clients:** The `/voice_commander` is shown with bolded action client connections pointing to two major servers:
    - **`/move_base`:** For navigation.
    - **`/move_group`:** For manipulation.
- **Low-Level Control:** `move_base` is shown publishing `/cmd_vel` topics, while `move_group` is shown publishing `FollowJointTrajectory` actions to the robot's controllers.

## Lab Instructions



### Lab 16.1: Building the Integrated System

**Reasoning:**
This is the capstone project. It is not about learning a single new concept, but about the skill of **system integration**. You will take the knowledge, and even some of the code, from all the previous labs and assemble it into a single, working application that is more than the sum of its parts.

**Instructions:**
This is a multi-week project. Break it down into testable milestones.

1.  **Milestone 1: The Simulation Environment**
    - In Gazebo, create a world file. Add a floor, walls, and a table.
    - Find a simulated mobile manipulator model (e.g., a TurtleBot3 with an OpenManipulator arm) and write a launch file to spawn it in your world.
    - Add a simple red sphere model to the world and place it on the table.

2.  **Milestone 2: The Perception Node**
    - Write (or adapt) an object detection node. For this project, you don't need a complex YOLO model. A simple Python script using OpenCV to find contours based on a red color threshold is sufficient.
    - Your node should subscribe to the robot's camera topic, find the red ball, and publish its `(x, y, z)` position. (You may need to use the depth camera or some geometry to estimate the 3D position from 2D pixel coordinates). Test this node in isolation.

3.  **Milestone 3: Navigation and Manipulation Setup**
    - **Navigation:** Run the SLAM lab again to create a map of your custom Gazebo world. Save the map. Create a launch file that starts the navigation stack (`amcl`, `move_base`) with your map. Test that you can send a navigation goal from Rviz to a spot near the table and have the robot drive there successfully.
    - **Manipulation:** Create the MoveIt! configuration package for your robot's arm. Launch the `demo.launch` file and test that you can use the Rviz plugin to plan and execute motions for the arm. Find and save the joint configurations for key poses like "home," "open gripper," "closed gripper," etc.

4.  **Milestone 4: The `voice_commander` Task Manager**
    - This is the core programming challenge. Write the central node that implements the state machine described above.
    - Use the `actionlib` client library in Python to write code that can send goals to the `move_base` and `move_group` action servers. Your node needs to send a goal, and then wait for the result before proceeding to the next state.
    - Integrate your speech recognition solution from the HRI chapter.

5.  **Milestone 5: Final Integration and Demonstration**
    - Create a single master launch file that starts everything: Gazebo, your perception node, your HRI nodes, and your `voice_commander`.
    - Run the launch file.
    - Speak the command.
    - Watch and debug as the robot executes the full sequence.

**Expected Outcome:**
Upon completing this project, you will have a working, complex robotics application that you have built and integrated yourself. You will have gained deep, practical experience in all major facets of modern robotics. This project is a powerful demonstration of your skills and serves as the perfect culmination of this course.

## Course Conclusion
Throughout these 16 chapters, we have journeyed from the basic components of a robot to the complexities of AI-driven behavior. We have seen that robotics is, above all else, a field of **system integration.** A successful robot is a harmonious symphony of perception, planning, control, and interaction. The open-source tools you have learned, particularly within the ROS ecosystem, are the building blocks for creating a new generation of intelligent machines. The journey doesn't end here. Continue to learn, to build, to experiment, and to contribute. The future of robotics is in your hands.
