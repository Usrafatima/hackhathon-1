---
title: "The Bridge: Connecting Python Agents to ROS 2 Controllers"
sidebar_position: 3
tags: [ROS 2, Python, rclpy, Robotics, Architecture, Bridge Pattern]
---

# Chapter 2: The Bridge - Connecting Python Agents to ROS 2 Controllers

In the world of modern robotics, development often happens in two distinct realms. On one side, we have the high-level world of **AI and Python-based logic**. This is where intelligent "agents" live—systems that might use advanced libraries like `transformers`, `langchain`, or custom machine learning models to make decisions. On the other side, we have the low-level world of **ROS 2 hardware controllers**, which expect precise, real-time commands in specific message formats.

The critical question is: how do we bridge these two worlds? How does a high-level Python agent, which might decide "move forward," communicate that intent to a ROS 2 controller that only understands `geometry_msgs/msg/Twist` messages on a `/cmd_vel` topic?

This chapter introduces the **Bridge Pattern**, a fundamental architectural solution for connecting high-level Python agents to the ROS 2 ecosystem using `rclpy`.

## 2.1 The "Two Worlds" Problem

Let's define our two realms more clearly:

-   **The High-Level Agent World:**
    -   **Environment:** Often a standard Python environment.
    -   **Logic:** Complex, often stateful, and may not operate in real-time. It thinks about *what* to do.
    -   **Output:** Typically simple, abstract commands like dictionaries (`{'direction': 'forward', 'speed': 0.5}`) or function calls.
    -   **Example:** A vision-based agent that analyzes a camera feed and decides, "I see a green bottle, I should move towards it."

-   **The Low-Level Controller World:**
    -   **Environment:** The ROS 2 graph.
    -   **Logic:** Real-time, highly reactive, and focused on *how* to execute an action.
    -   **Input:** Expects a continuous stream of specific ROS 2 messages, like `Twist` for velocity or `JointTrajectory` for arm movements.
    -   **Example:** A mobile base controller that needs a `Twist` message every 100ms to keep the robot moving smoothly.

Directly integrating ROS 2 code into the high-level agent can make the agent code messy, difficult to test in isolation, and overly dependent on the ROS 2 environment. The solution is to create a dedicated bridge.

## 2.2 The Bridge Node: An Architectural Pattern

The **Bridge Node** is a specialized ROS 2 node whose sole purpose is to act as an adapter or translator between the high-level agent and the low-level ROS 2 controllers.

<!-- ![Figure 2-1: Agent-Controller Bridge Architecture](img/agent_controller_architecture.png) -->
*A diagram showing a Python Agent's abstract commands being translated by a dedicated "Bridge Node" into specific ROS 2 messages, which are then sent to the robot's hardware controller nodes.*

This architecture provides a clean separation of concerns:
-   **The Agent** remains pure Python. It can be developed and tested independently of any ROS 2 system.
-   **The Bridge Node** handles all ROS 2 communication. It subscribes to necessary sensor topics to feed data *to* the agent and publishes command topics to send instructions *from* the agent.

## 2.3 Implementing the Bridge: A Practical Example

Let's build a bridge for a hypothetical navigation agent.

**Step 1: Define the High-Level Agent**
Our agent is a simple Python class. Its job is to decide the robot's next move. For this example, it will just command the robot to spin in a circle.

```python
# file: simple_nav_agent.py

class NavigationAgent:
    """
    A simple high-level agent that decides the robot's velocity.
    This class has NO ROS 2 dependencies.
    """
    def __init__(self):
        self.state = "initializing"
    
    def get_next_command(self):
        """
        The core logic of the agent. Returns the next desired velocity.
        """
        # In a real system, this could involve a complex calculation,
        # a neural network inference, or user input.
        command = {
            'velocity_x': 0.0,  # No forward movement
            'angular_z': 0.5    # Spin counter-clockwise at 0.5 rad/s
        }
        return command
```

**Step 2: Understand the Low-Level Controller Interface**
Our robot's mobile base controller is a standard ROS 2 node that listens on the `/cmd_vel` topic. It expects messages of type `geometry_msgs/msg/Twist`. The structure of a `Twist` message is:
- `linear`: A vector with `x`, `y`, `z` components.
- `angular`: A vector with `x`, `y`, `z` components.

Our bridge must convert the agent's dictionary command into this `Twist` message format.

**Step 3: Build the Bridge Node**
This node imports the agent, creates a ROS 2 publisher, and runs a loop to continuously translate and publish commands.

```python
# file: agent_bridge_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Import our high-level agent
from simple_nav_agent import NavigationAgent

class AgentBridgeNode(Node):
    def __init__(self):
        super().__init__('agent_bridge_node')
        
        # 1. Instantiate the high-level agent
        self.agent = NavigationAgent()
        self.get_logger().info("Navigation Agent instantiated.")

        # 2. Create the low-level ROS 2 publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. Create a timer to run the bridge loop at a fixed rate (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.bridge_loop)

    def bridge_loop(self):
        """
        This is the core of the bridge. It gets a command from the agent,
        translates it, and publishes it to the ROS 2 graph.
        """
        # Get the high-level command from the agent
        agent_command = self.agent.get_next_command()
        
        # Create a low-level ROS 2 message
        twist_msg = Twist()
        
        # Translate the dictionary command into the message format
        twist_msg.linear.x = float(agent_command.get('velocity_x', 0.0))
        twist_msg.angular.z = float(agent_command.get('angular_z', 0.0))
        
        # Publish the ROS 2 message
        self.velocity_publisher.publish(twist_msg)
        
        # Optional: Log the action
        self.get_logger().info(f"Published cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    bridge_node = AgentBridgeNode()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Bi-Directional Communication

This pattern is not limited to one-way commands. The Bridge Node can also subscribe to ROS 2 topics (like `/odom` for odometry or `/scan` for laser data) and pass that information *to* the agent, allowing the agent to make stateful, informed decisions.

```python
# Inside the Bridge Node...

# Create a subscriber to feed data TO the agent
# self.odom_subscriber = self.create_subscription(
#     Odometry, '/odom', self.odom_callback, 10
# )

# def odom_callback(self, msg):
#     # Update the agent's internal state with the latest odometry
#     self.agent.update_state(position=msg.pose.pose.position)
```

## 2.4 Summary: A Clean and Testable Architecture

By adopting the Bridge Pattern, you create a system that is clean, modular, and highly testable.
- Your **Agent** can be tested with standard Python unit tests, completely independent of ROS 2.
- Your **ROS 2 Controllers** can be tested with standard ROS 2 tools.
- Your **Bridge Node** can be tested by providing a mock agent and checking its ROS 2 publications.

This separation is the key to managing the complexity of intelligent robots and is a foundational pattern you will use throughout your robotics journey.