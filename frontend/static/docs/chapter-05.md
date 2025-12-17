---
title: 'Chapter 5: Humanoid Locomotion and Stability'
metadata:
  chapter_number: 5
  keywords: ['locomotion', 'bipedal walking', 'stability', 'zmp', 'zero moment point', 'inverted pendulum', 'gait']
---

# Chapter 5: Humanoid Locomotion and Stability


## Goals


- **To explain the fundamental challenges of bipedal walking.** We will explore why making a robot walk on two legs is one of the most difficult problems in robotics.
- **To introduce the key concepts and models used for generating stable walking patterns,** with a special focus on the **Zero Moment Point (ZMP)**.
- **To discuss the critical difference between static and dynamic stability** and how it relates to the way robots and humans walk.

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **Explain why bipedal walking is an inherently unstable process** and list the key difficulties involved.
- **Define the Zero Moment Point (ZMP)** and explain its "golden rule" for maintaining balance.
- **Contrast statically stable walking with dynamically stable walking,** providing an example of each.

## Full Explanations of Concepts



### The Challenge of Bipedal Locomotion

Walking on two legs is something most humans do without conscious thought, but for a robot, it is an immense challenge. Bipedal walking is often described as a process of "controlled falling."

- **High Center of Gravity and Small Support Base:** A humanoid robot, like a human, has a high center of gravity and a very small base of support (the area covered by its feet). This makes it inherently top-heavy and prone to tipping over. Compare this to a car, which has a low center of gravity and a very wide base of support.
- **The Problem of Underactuation:** A robot can directly control the angles of its joints (it has motors there), but it cannot directly control the position or orientation of its torso in space. It can only influence its torso *indirectly* by moving its legs and shifting its weight. This indirectness makes control much more complex.
- **Constant Sensing and Adjustment:** Unlike a simple factory robot that repeats the same motion, a walking robot must constantly sense its own state (balance, orientation) and the state of the world (the slope of the ground, unexpected bumps) and make millisecond-fast adjustments to stay upright.

### Static vs. Dynamic Stability

The way a robot walks can be categorized based on how it manages its stability.

#### Statically Stable Walking
- **Principle:** The robot's Center of Gravity (COG) is maintained over its support polygon *at all times*. The support polygon is the convex hull of all the points where the robot is touching the ground.
- **How it works:** Before lifting a foot, the robot first shifts its entire body weight over the other foot. It then slowly lifts the swing leg, moves it forward, places it down, and only then begins to shift its weight again. At no point is the robot ever in danger of falling, even if it were to freeze mid-motion.
- **Analogy:** A person walking on a slippery, icy path takes very slow, deliberate, wide-stanced steps. Early robots like WABOT-1 also walked this way.
- **Characteristics:**
    - **Pros:** Very simple to control and guaranteed to be stable.
    - **Cons:** Extremely slow, energy-inefficient, and looks very unnatural. It requires the robot to have very large feet to ensure a stable support polygon.

#### Dynamically Stable Walking
- **Principle:** The robot's COG is allowed to move outside the support polygon during the gait cycle. The robot uses its own momentum and the precise placement of its next step to "catch" itself from a fall.
- **How it works:** The robot initiates a step by actively letting itself fall forward. Its momentum carries its COG outside of its support foot. It then swings its other leg forward and places it in just the right spot to establish a new base of support and prevent the fall, seamlessly transitioning into the next step.
- **Analogy:** This is how humans walk and run. It is a continuous process of falling and catching ourselves.
- **Characteristics:**
    - **Pros:** Fast, energy-efficient, and natural-looking. It allows for running, jumping, and agile maneuvers.
    - **Cons:** Requires extremely sophisticated and fast control systems. The robot is always just one step away from falling over.

### The Zero Moment Point (ZMP): The Key to Balance

For dynamically stable walking, the most important concept is the Zero Moment Point or ZMP.

- **Definition:** The Zero Moment Point is the point on the ground surface where the net moment of all inertial and gravitational forces acting on the robot is zero. A more intuitive way to think about it is as the **Center of Pressure (CoP)** under the robot's feet. If the robot were standing on a giant force-sensing plate, the ZMP would be the point where the total force is concentrated.
- **The Golden Rule of Bipedal Walking:** **To maintain balance, the ZMP must always remain within the support polygon.** If the ZMP moves outside the support polygon, the robot will experience an unrecoverable tipping moment and will fall over.
- **Control using ZMP:** Modern humanoid control is built around this principle.
    1.  **Planning:** A high-level planner generates a desired trajectory for the robot's Center of Mass (COM) and a corresponding trajectory for its ZMP. For straight walking, the ZMP trajectory would be a path that moves back and forth between the centers of the left and right feet.
    2.  **Sensing:** Force sensors in the robot's feet measure the actual, real-world ZMP location.
    3.  **Feedback Control:** A controller compares the *desired* ZMP location to the *actual* ZMP location. If there is an error, the controller makes tiny adjustments to the robot's motion (e.g., leaning the torso, bending the ankles) to push the actual ZMP back towards its desired location.

### Walking Pattern Generation

How does the controller come up with the desired ZMP and COM trajectories in the first place? It uses simplified physical models.

- **The Inverted Pendulum Model:** The core model used in bipedal walking. The robot is simplified to a single point mass (its Center of Mass) balanced on top of a massless, rigid leg of a certain length. The physics of this system are well-understood.
- **Linear Inverted Pendulum Mode (LIPM):** A further simplification where the height of the Center of Mass is assumed to be constant. This makes the equations of motion linear and very easy to solve quickly. The LIPM is the workhorse of walking pattern generation. It allows the planner to rapidly calculate a smooth trajectory for the COM that results in a ZMP trajectory that stays within the desired bounds.
- **The Gait Cycle:** A single cycle of walking is broken down into phases:
    - **Single Support Phase:** One foot is on the ground (the "stance foot") and the other is in the air (the "swing foot"). This is where the robot is "falling."
    - **Double Support Phase:** Both feet are on the ground. This is a brief, stable phase where the robot shifts its weight from the previous stance foot to the new one.

## Step-by-Step Diagram Explanation

### ZMP and the Support Polygon

![ZMP and the Support Polygon](https://i.imgur.com/example-diagram.png)
*(Note: Replace with a real diagram URL showing the ZMP and support polygon)*

This diagram provides a top-down view illustrating the "golden rule" of ZMP.

1.  **Left Panel (Stable):**
    - **Support Polygon:** The outlines of the robot's left and right feet are shown on the ground. The area encompassing both feet and the space between them is shaded and labeled "Support Polygon."
    - **Center of Gravity (COG):** The projection of the robot's center of gravity onto the ground is shown as a circle, located somewhere between the two feet.
    - **Zero Moment Point (ZMP):** A cross or star symbol labeled "ZMP" is shown. Critically, it is located safely **inside** the boundaries of the support polygon.
    - **Caption:** "Stable: ZMP is inside the support polygon."

2.  **Right Panel (Unstable / Tipping):**
    - **Support Polygon:** The robot is in a single-support phase, so the polygon is only the area of one foot.
    - **Center of Gravity (COG):** The COG projection has moved far forward, ahead of the support foot.
    - **Zero Moment Point (ZMP):** The ZMP has moved to the very front edge of the support foot. Any further forward motion of the COG will cause the ZMP to leave the polygon.
    - **Caption:** "Unstable: ZMP has reached the boundary. The robot will tip over if no corrective action is taken (e.g., taking another step)."

## Lab Instructions



### Lab 5.1: Visualizing the ZMP in a Walking Simulation

**Reasoning:**
The concept of the ZMP can feel abstract. This lab makes it concrete by allowing you to see the ZMP move in real-time as a simulated robot walks and responds to disturbances. It builds a powerful intuition for how a humanoid robot *feels* its own balance.

**Instructions:**
1.  **Find a Simulator:** You will need a humanoid robot simulator that can visualize the ZMP. **Choreonoid** is an excellent open-source option designed for this. Alternatively, **Gazebo** with specific plugins can also provide this visualization. Search for "ZMP visualization humanoid simulator."
2.  **Load and Walk:** Load a humanoid robot model (e.g., the HRP-2 or similar) and use the simulator's interface to command it to walk forward.
3.  **Enable ZMP Visualization:** Find the setting to display the support polygon and the Zero Moment Point (it might also be called Center of Pressure or CoP). The ZMP is typically shown as a moving dot, cross, or arrow.
4.  **Observation 1: Normal Walking**
    - Watch the robot walk in slow motion.
    - Observe the ZMP's path. As the robot prepares to lift its left foot, you will see the ZMP slide across the ground until it is entirely under the right foot.
    - As the left foot swings forward, the ZMP will move around under the right foot to maintain balance.
    - Just before the left foot lands, the ZMP will move towards the front of the right foot. As soon as the left foot makes contact, the support polygon instantly becomes larger, and the ZMP can now travel across to the left foot to start the next step.
5.  **Observation 2: The "Push" Test**
    - While the robot is standing still (in double support), find the feature in the simulator to apply an external force or "push" to the robot's torso.
    - **Action:** Apply a small, brief push from the side.
    - **Observe:** You will see the ZMP immediately shift in the direction *opposite* to the push as the robot's control system counteracts the force. The robot will likely move its ankles and hips (this is called an "ankle strategy" or "hip strategy") to reposition its center of mass and push the ZMP back to the center.
    - **Action:** Apply a very large push.
    - **Observe:** The ZMP will shoot to the edge of the support polygon. The robot's strategy will fail, the ZMP will exit the polygon, and the robot will lose its balance and begin to fall.

**Expected Outcome:**
- Students will no longer see walking as just "moving legs." They will see it as a delicate dance of "managing the ZMP."
- They will understand that all the seemingly complex swaying of a robot's upper body is not random; it is a calculated motion with the precise goal of keeping the ZMP within the safe area of the support polygon.
