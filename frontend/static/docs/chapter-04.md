---
title: 'Chapter 4: Robot Control Architectures'
metadata:
  chapter_number: 4
  keywords: ['robot control', 'control architecture', 'pid controller', 'feedback control', 'reactive control', 'deliberative control', 'hybrid control']
---

# Chapter 4: Robot Control Architectures


## Goals


- **To introduce the fundamental principles of robot control.** We will explore the core concepts that enable a robot to perform actions reliably and accurately.
- **To describe and contrast different control architectures.** We will look at the evolution of strategies for organizing a robot's "brain," from simple reflexes to complex planning.
- **To explain the role of feedback control and the PID controller,** the ubiquitous workhorse of robotics.

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **Explain the critical difference between open-loop and closed-loop (feedback) control** and why the latter is essential for robotics.
- **Describe the three main types of robot control architectures** (Reactive, Deliberative, Hybrid) and their respective strengths and weaknesses.
- **Explain the function of the Proportional, Integral, and Derivative terms** in a PID controller and how they work together to achieve stable control.

## Full Explanations of Concepts



### Open-Loop vs. Closed-Loop Control

The most fundamental concept in control theory is the distinction between sending a command blindly and sending a command while actively watching the result.

#### Open-Loop Control
This is the simplest form of control. The controller sends a command to an actuator without any feedback to confirm the outcome.
- **How it works:** `Controller → Command → Actuator`
- **Analogy:** You close your eyes and throw a ball, *assuming* it will hit the target.
- **Example:** A simple kitchen toaster. You set the timer (the command), and it applies heat for that duration. It doesn't have a sensor to check if the toast is perfectly browned; it just runs for the time you specified.
- **Problem:** It is extremely sensitive to disturbances and errors. If the motor is under a heavier load than expected, or if it slips, the final state will be incorrect, and the controller will never know. It is not robust.

#### Closed-Loop (Feedback) Control
This is the dominant paradigm in all modern robotics. The controller uses sensor feedback to continuously measure the difference between the desired state and the actual state and adjusts its output accordingly.
- **How it works:** `Controller → Command → Actuator → System State → Sensor → Feedback → Controller` (The loop is "closed").
- **Analogy:** You watch the ball as you throw it, constantly adjusting your arm's motion mid-throw to ensure it hits the target.
- **Example:** The cruise control in a car.
    - **Desired State:** 60 mph.
    - **Actual State:** Measured by the speedometer (the sensor).
    - **Error:** `Desired State - Actual State`. If the car is going 58 mph up a hill, the error is +2 mph.
    - **Control Action:** The controller detects this error and gives the engine more gas to compensate. If the car starts going downhill and speeds up to 62 mph (error = -2 mph), the controller reduces the throttle.

### The PID Controller: The Workhorse of Robotics

The Proportional-Integral-Derivative (PID) controller is by far the most common type of feedback controller in robotics and industrial automation. Its goal is to minimize the `error` over time by adjusting its control output. It's powerful because it looks at the error in three ways: its present, its past, and its future.

Let `e(t)` be the error at time `t`.

#### P: Proportional Term
The controller output is directly proportional to the current error.
- **Action:** `Output = Kp * e(t)`
- **Behavior:** This is the primary corrective force. If you are far from your goal, you push hard. If you are close, you push lightly.
- **Limitation:** A P-only controller often results in a **steady-state error**. For example, a robot arm trying to hold a heavy weight might stop just short of its target position because the force from the P term exactly balances the force of gravity. The controller is "happy" with this small, persistent error.

#### I: Integral Term
The controller output is proportional to the accumulated error over time.
- **Action:** `Output = Ki * ∫e(t)dt`
- **Behavior:** This term looks at the "past" error. If there is a small, persistent steady-state error (like the one above), the integral of that error grows over time. Eventually, the output from the I-term becomes large enough to overcome the force of gravity and eliminate the steady-state error.
- **Limitation:** The I-term can cause the system to "overshoot" its target because it builds up momentum. This can lead to oscillations.

#### D: Derivative Term
The controller output is proportional to the rate of change of the error.
- **Action:** `Output = Kd * de(t)/dt`
- **Behavior:** This term looks at the "future" error. If the error is changing rapidly, it means the system is approaching the target too fast. The D-term applies a braking force to dampen this velocity and prevent overshoot. It makes the system more stable and less prone to oscillation.
- **Analogy:** As you drive your car towards a stop sign, you start easing off the gas and applying the brakes *before* you reach the sign, because you can see you are approaching it quickly. That's the derivative action.

By "tuning" the three constants (`Kp`, `Ki`, `Kd`), an engineer can achieve a controller that is fast, stable, and accurate for a specific system.

### Control Architectures: Organizing the Brain

A control architecture is the high-level design that organizes how a robot perceives, decides, and acts.

#### 1. Reactive Architectures
- **Principle:** A direct, tight coupling between sensors and actuators. The system is composed of a collection of simple "behaviors."
- **Example:** The **Subsumption Architecture**, developed by Rodney Brooks. Behaviors are layered, like `[1. Move Forward]`, `[2. If Obstacle Detected, Turn Away]`, `[3. If Landmark Seen, Turn Towards It]`. A higher-level behavior (like #2) can *subsume* (override) a lower-level one (like #1).
- **Pros:** Extremely fast and responsive. Very robust in dynamic, cluttered environments.
- **Cons:** No memory, no planning, no concept of a long-term goal. It's purely reflexive. It cannot do something simple like "go out of the room" because it doesn't know what a "room" is.

#### 2. Deliberative Architectures (Sense-Plan-Act)
- **Principle:** The classic AI approach. The robot first builds a complete model of the world, then uses that model to create a perfect, detailed plan from start to finish, and only then begins to execute that plan.
- **How it works:**
    1.  **Sense:** Collect all possible sensor data.
    2.  **Plan:** Use a complex algorithm to find an optimal sequence of actions to achieve a goal.
    3.  **Act:** Execute the first step of the plan. (And often, discard the rest of the plan and start over).
- **Pros:** Can solve complex problems that require foresight and optimization.
- **Cons:** Extremely slow. By the time the robot has finished "thinking" of a plan, the world has often changed, making the plan useless. This is known as **The Frame Problem**. This architecture is not suitable for dynamic environments.

#### 3. Hybrid Architectures (The Modern Standard)
- **Principle:** Combines the best of both worlds. A slower, deliberative layer sets high-level goals, while a fast, reactive layer handles the low-level execution and responds to immediate environmental changes.
- **Structure:** Usually composed of three layers:
    1.  **The Deliberative Layer (Planner):** This is the "thinker." It operates on a map or world model and makes high-level, slow-paced decisions. (e.g., "The goal is to get the water bottle from the kitchen. The best route is through the hallway.")
    2.  **The Executive Layer (Sequencer):** This layer acts as a bridge. It takes the high-level plan and breaks it down into a sequence of smaller, manageable tasks for the reactive layer. (e.g., "1. Execute 'Exit Room'. 2. Execute 'Navigate Hallway'. 3. Execute 'Enter Kitchen'.")
    3.  **The Reactive Layer (Controller):** This is the "doer." It operates in real-time, executing the current task (e.g., "Move forward at 0.5 m/s") while using sensor feedback to handle immediate concerns (e.g., "Dodge the chair!"). It is often a collection of reactive behaviors.

This hybrid model allows a robot to pursue long-term goals while remaining responsive to the immediate, unpredictable nature of the real world.

## Step-by-Step Diagram Explanation

### Hybrid Control Architecture

![Hybrid Control Architecture](https://i.imgur.com/example-diagram.png)
*(Note: Replace with a real diagram URL showing the hybrid architecture)*

This diagram shows the typical three-layered structure of a hybrid control system.

1.  **Top Layer (Deliberative):**
    - A box labeled **"Planner"** or **"Deliberative Layer"**.
    - It receives slow inputs: the high-level **"Goal"** from the user and the overall **"World Model / Map"**.
    - It produces a **"High-Level Plan"** (e.g., a series of waypoints on a map).
2.  **Middle Layer (Executive):**
    - A box labeled **"Executive"** or **"Sequencer"**.
    - It takes the `High-Level Plan` as input.
    - It outputs a **"Sequence of Sub-tasks"** (e.g., `[Go to Doorway]`, `[Traverse Hall]`). It sends one sub-task at a time to the layer below.
3.  **Bottom Layer (Reactive):**
    - A box labeled **"Reactive Controller"**.
    - This layer is shown inside a fast feedback loop. It takes the current `Sub-task` from the executive.
    - It receives real-time **"Sensor Data"** (e.g., from LiDAR and IMU).
    - It generates the final **"Motor Commands"** (`Velocities`, `Torques`) that go to the robot's actuators.
    - This loop is very fast, running hundreds of times per second to ensure the robot is stable and avoids immediate obstacles.
4.  **Feedback Loops:**
    - A **fast loop** connects the sensors, reactive layer, and motors.
    - A **slow loop** shows information from the sensors being used to update the `World Model / Map` used by the top deliberative layer.

## Lab Instructions



### Lab 4.1: Tuning a PID Controller for a Self-Balancing Robot

**Reasoning:**
There is no better way to understand PID control than to tune one yourself. A two-wheeled self-balancing robot is a classic "inverted pendulum" problem, where feedback control is not just helpful, but absolutely essential for stability. This lab provides a powerful, intuitive feel for how each gain affects the system's behavior.

**Instructions:**
1.  **Find a Simulator:** Search for a "self-balancing robot simulator," "PID control demo," or "inverted pendulum simulator." Many web-based and downloadable options are available. The simulator should show a two-wheeled robot and provide sliders for the `Kp`, `Ki`, and `Kd` gains.
2.  **The Goal:** The robot's goal is to keep its body perfectly upright (tilt angle = 0). The PID controller's input is the measured tilt angle (the error), and its output is the command sent to the wheel motors (e.g., speed or torque).
3.  **Step 1: P-Only Control**
    - Set the `Ki` and `Kd` gains to `0`.
    - Set `Kp` to a very small value. Give the robot a "push" in the simulator. It will fall over.
    - **Action:** Gradually increase `Kp`. You will reach a point where the robot starts to correct itself. It will likely oscillate back and forth wildly. Find a `Kp` value where it is *almost* stable but still oscillates. If you increase `Kp` too much, the oscillations will grow until it becomes violently unstable.
4.  **Step 2: PD Control**
    - Leave `Kp` at the value you found.
    - **Action:** Now, slowly start to increase the `Kd` gain. You will immediately see the oscillations begin to shrink. The D term is "damping" the system, acting as a virtual brake to fight the overshoot caused by the P term.
    - **Tune:** Adjust `Kp` and `Kd` together. A higher `Kp` gives a "stronger" correction, while a higher `Kd` gives more "damping." Try to find a combination where the robot quickly returns to vertical after a push with minimal wobbling.
5.  **Step 3: PID Control**
    - With your PD controller tuned, give the robot a push. It should be stable, but you might notice it settles with a very slight, persistent tilt (a steady-state error).
    - **Action:** Slowly increase the `Ki` gain. The I term will notice this persistent error and build up a corrective force over time, making the robot return to perfectly vertical.
    - **Warning:** Be careful with the `Ki` term! Too much integral gain can cause the system to become unstable again, creating slow, long-period oscillations or causing the robot to "run away."

**Expected Outcome:**
Students will have a powerful visual and interactive understanding of the role of each PID gain:
- **P** provides the primary restoring force.
- **D** provides damping to fight oscillations and improve stability.
- **I** corrects for small, long-term, steady-state errors.
They will have gained a practical feel for the "art" of PID tuning.
