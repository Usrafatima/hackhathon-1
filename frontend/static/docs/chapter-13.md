---
title: 'Chapter 13: Human-Robot Interaction (HRI)'
metadata:
  chapter_number: 13
  keywords: ['human-robot interaction', 'hri', 'social robotics', 'speech recognition', 'gestures', 'safety']
---

# Chapter 13: Human-Robot Interaction (HRI)


## Goals


- **To introduce Human-Robot Interaction (HRI)** as a critical, multidisciplinary field that bridges robotics with human-centered design.
- **To discuss the key communication channels** between humans and robots, including both verbal and non-verbal modalities.
- **To explore the concepts of social robotics, safety, and ethics** that are paramount when designing robots meant to operate in human spaces.

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **Define HRI** and list its primary goals (usability, usefulness, social acceptance).
- **Describe at least two verbal and two non-verbal methods** of HRI.
- **Discuss the importance of safety, legibility, and predictability** in robots designed to interact with humans.

## Full Explanations of Concepts



### What is Human-Robot Interaction?

As robots move out of cages on factory floors and into our homes, offices, and hospitals, the way they interact with people becomes as important as their ability to perform tasks.

**Definition:** Human-Robot Interaction (HRI) is a field of study dedicated to understanding, designing, and evaluating robotic systems for use by, with, and for humans.

HRI is inherently multidisciplinary, lying at the intersection of:
- **Robotics:** The physical hardware, control systems, and autonomy.
- **Artificial Intelligence:** The robot's ability to perceive, reason, and learn.
- **Social Sciences:** Psychology, sociology, and communication studies that inform how humans perceive and react to robots.
- **Design:** User experience (UX) and industrial design that shape the robot's form and interface.

The ultimate goal of HRI is to create robots that are not just functional, but also **useful, usable, and socially acceptable.**

### Channels of Communication

For a human and a robot to collaborate effectively, they must be able to communicate. This happens through multiple channels, many of which are subconscious for humans.

#### Verbal Communication (Input and Output)

This involves a full pipeline to process spoken language:
1.  **Automatic Speech Recognition (ASR):** The robot's microphone captures human speech and a software model (like OpenAI's Whisper or Google's Speech-to-Text) converts the audio into a string of text. This is the "ears" of the robot.
2.  **Natural Language Understanding (NLU):** The robot then parses this text to understand the user's *intent*. For example, both "Can you grab that bottle for me?" and "Please fetch the bottle" should map to the same `FETCH(bottle)` intent.
3.  **Dialog Management:** This is the robot's "brain" in a conversation. It tracks the context of the conversation and decides what action to take or what to say next.
4.  **Text-to-Speech (TTS):** Once the robot has decided on a response (e.g., "Certainly, I am getting the bottle now"), a TTS engine converts that text into audible, synthesized speech.

#### Non-Verbal Communication

For HRI, non-verbal cues are often more powerful and important than language for creating a smooth and intuitive interaction.

- **Gestures:** Humans use gestures constantly. A robot should be able to understand a person pointing to an object of interest. Conversely, the robot can use its own arms to create gestures, such as pointing to a location to guide a person.
- **Gaze:** Where a robot is "looking" is a powerful signal of its attention and intent. By directing its head or cameras towards a person, it signals that it is listening. By looking at an object before moving towards it, it signals its intention, making its actions predictable.
- **Proxemics:** This is the study of the social use of space. A robot must be programmed to respect personal space. If it gets too close to a person, it can be intimidating; if it stays too far away, it can seem aloof. The appropriate distance depends on the cultural context and the task.
- **Legibility and Predictability:** A robot's actions must be **legible**—a person should be able to watch its movements and intuitively understand what it is trying to do. For example, if a robot is going to pick up a cup, it shouldn't take a strange, circuitous route. It should move its arm in a direct, understandable way. Predictability builds trust; a robot that behaves consistently is one that people feel comfortable around.

### Social Robotics

**Definition:** A subfield of HRI focused on robots designed to fulfill social roles and adhere to social norms. These robots are often companions, tutors, guides, or assistants.

- **Expressing Emotion:** Social robots often need to express "emotions" to be effective. This can be done through animated facial expressions on a screen, mechanical movements of an artificial face (like Sophia the Robot), changes in the color of LED lights, or by modulating the tone of their synthesized voice.
- **Building Trust:** Trust is the cornerstone of HRI. A robot builds trust by being reliable, predictable, and competent. If a robot consistently fails at its task or behaves erratically, users will quickly lose trust in it.
- **The Uncanny Valley:** This is a famous hypothesis in HRI. It states that as a robot's appearance becomes more and more human-like, our affinity for it increases, but only up to a certain point. When a robot is *almost* perfectly human but has subtle flaws (e.g., unnatural eye movement, stiff facial expressions), our affinity plummets, and the robot becomes deeply unsettling or "uncanny." This is a major challenge for designers of realistic humanoids.

### Safety in HRI

Safety is the absolute number one priority when a robot is not in a cage.

- **Physical Safety:**
    - **Collision Avoidance:** The robot's perception and planning systems must be robust enough to never collide with a person, even if the person moves unexpectedly.
    - **Collaborative Robots (Cobots):** A special class of industrial robots designed to work alongside humans. They are typically lightweight and equipped with sensitive force/torque sensors in their joints. If they make contact with a person, they detect the force and immediately stop, preventing injury.
    - **Soft Robotics:** Building robots from soft, compliant materials is another strategy to make them inherently safe. A soft gripper is far less likely to cause injury than a metal one.
- **Psychological Safety:** A robot shouldn't just be physically safe; it should *feel* safe to be around. A large, fast-moving robot can be psychologically distressing even if it never makes contact. Its movements should be smooth and predictable, and its behavior should not be startling.

## Step-by-Step Diagram Explanation

### The Human-Robot Interaction Loop

![The HRI Loop](https://i.imgur.com/example-diagram.png)
*(Note: Replace with a diagram of the HRI loop)*

This diagram shows the continuous cycle of action and reaction between a human and an interactive robot.

1.  **Human Action:** A person performs an action. This can be **verbal** (speaking a command) or **non-verbal** (pointing to an object, looking at the robot).
2.  **Robot Perception:** The robot's sensors capture the human's action. Its **microphone** captures the speech, and its **cameras** capture the gesture and the person's pose.
3.  **Robot "Brain" (Planning & Decision-Making):** The robot's AI and control systems process the perceptual information. The ASR node converts speech to text, the NLU node determines intent, and the computer vision system identifies the gesture. A central behavior-planning module (like a Behavior Tree) decides what to do next based on this input.
4.  **Robot Action:** The robot executes the planned action. This action is itself a form of communication. It can be a **verbal response** ("Okay, I will pick up the red block") or a **physical, non-verbal action** (moving its arm towards the block).
5.  **Legible Feedback:** Crucially, the robot's action is designed to be **legible** and **predictable**. Its motion clearly signals its intent. This action is then observed by the human.
6.  **Human Perception:** The human sees the robot's action, understands its intent, and can then provide the next input, thus completing the loop.

A smooth HRI experience is one where this loop is fast, intuitive, and feels natural to the human participant.

## Lab Instructions



### Lab 13.1: Voice Control of a Simulated Robot

**Reasoning:**
This lab provides a practical, end-to-end demonstration of a core verbal HRI pipeline. You will connect a speech recognition system to a robot's motor control, allowing you to command a robot with your voice.

**Instructions:**
1.  **Prerequisites:** A working ROS installation, the TurtleBot3 Gazebo simulation from previous labs, and a microphone connected to your computer.
2.  **Install a Speech Recognition Package:** There are several options for ROS. A good, simple one to start with is `pocketsphinx`, which runs offline.
    ```bash
    sudo apt-get install ros-<distro>-pocketsphinx
    ```
    Alternatively, you could write a node that uses a cloud-based API like Google or Whisper.
3.  **Configure `pocketsphinx`:** This package requires a simple "dictionary" and "language model" file to tell it which words to listen for. Create a text file `commands.txt` with your desired commands, one per line:
    ```
    FORWARD
    BACKWARD
    TURN LEFT
    TURN RIGHT
    STOP
    ```
    You will then use an online tool to convert this into the dictionary (`.dic`) and language model (`.lm`) files that pocketsphinx needs.
4.  **Create a Voice Control Node:**
    - Write a new Python ROS node named `voice_commander.py`.
    - This node will **subscribe** to the `/recognizer/output` topic (where the `pocketsphinx` node publishes recognized text as `std_msgs/String`).
    - This node will **publish** `geometry_msgs/Twist` messages to the `/cmd_vel` topic to control the TurtleBot3.
    - In your subscriber's callback function:
        - Get the recognized text from the message (`msg.data`).
        - Create a new `Twist` message.
        - Use an `if/elif/else` block to check the text:
            - If "FORWARD" is in `msg.data`, set `twist.linear.x = 0.5`.
            - If "TURN LEFT" is in `msg.data`, set `twist.angular.z = 1.0`.
            - If "STOP" is in `msg.data`, set all values to `0.0`.
        - Publish the `twist` message.
5.  **Run the System:**
    - **Terminal 1:** Launch the TurtleBot3 simulation: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`.
    - **Terminal 2:** Launch the speech recognizer, telling it where to find your custom dictionary and language model files.
    - **Terminal 3:** Launch your voice commander node: `rosrun your_package voice_commander.py`.
6.  **Test It:**
    - Speak the commands clearly into your microphone.
    - **Observe:** The robot in Gazebo should move according to your voice commands. You will likely see some recognition errors, which is typical of ASR systems.

**Expected Outcome:**
- Students will have built a complete, voice-activated HRI system.
- They will understand the modularity of ROS, where one node (pocketsphinx) handles the complex task of perception, and a second, much simpler node (voice_commander) handles the task of translating that perception into action.
- This provides a concrete foundation for understanding how more complex verbal interactions can be built.
