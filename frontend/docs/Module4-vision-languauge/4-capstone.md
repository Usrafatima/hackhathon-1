---
title: "Capstone Project: The Autonomous Humanoid"
sidebar_position: 4
tags: [Capstone, VLA, HRI, System Integration, ROS 2, LLM]
---

# Chapter 14: Capstone Project: The Autonomous Humanoid

We have arrived at the final chapter, the culmination of our entire journey. Here, we will integrate the concepts from every preceding module—the robot's physical definition, its simulated world, its ROS 2 nervous system, its perceptive senses, and its cognitive LLM brain—into a single, functioning, autonomous system. This capstone project will bring our humanoid to life, enabling it to respond to a spoken command by navigating its environment, identifying an object, and performing a task.

### Comprehensive Introduction: Weaving the Threads Together

This project is more than just a final example; it is a masterclass in **systems integration**, which is the most critical and often most challenging aspect of real-world robotics. A robot is not a single algorithm but a complex ecosystem of dozens of independent software nodes that must communicate and cooperate flawlessly. This chapter will provide a blueprint for this integration, focusing on a central "Orchestrator" node that manages the robot's state and directs the flow of information between all the subsystems we have built. We will walk through the complete data flow, from a human voice vibrating the air to the final, precise movements of the robot's hand.

---

## System Architecture: Connecting the Modules

Before writing any code, we must visualize the complete system architecture. This diagram shows how all our custom nodes and standard ROS 2 packages work in concert.

<!-- ![Figure 14-1: Final System Architecture Diagram](img/capstone_architecture.png) -->
*This diagram shows the complete data flow of the capstone project. The `Orchestrator` node acts as the central brain, receiving voice commands, dispatching planning requests to the `LLM Planner`, and sending action goals (like navigation and manipulation) to the appropriate ROS 2 action servers (Nav2, MoveIt). It receives feedback and perception data from the sensor and SLAM pipelines.*

### Table 1: Module Integration Breakdown

| Module | Concepts Used                                     | Role in Capstone Project                                                                                                         |
| :----- | :------------------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------- |
| **1**  | URDF, ROS 2 Nodes, Topics, Services, Actions        | Defines the robot's physical structure and provides the fundamental ROS 2 communication patterns for all nodes.                     |
| **2**  | Gazebo Simulation, Sensor/Physics Configuration   | Creates the virtual "test world" where the robot will live, complete with physics, gravity, and simulated sensors.                    |
| **3**  | Isaac ROS (VSLAM), Nav2 (Path Planning)           | VSLAM provides the robot's pose and map. Nav2 provides the `NavigateToPose` action server for autonomous point-to-point navigation. |
| **4**  | Whisper, LLM Planner, Action Space              | Provides the HRI and cognitive layers: `Whisper` transcribes voice, and the `LLM Planner` decomposes commands into executable steps. |

---

## The Main Orchestrator Node: The Robot's "Consciousness"

The heart of our system is a central state machine that we will call the `Orchestrator`. This node doesn't perform any single task like SLAM or transcription; instead, its sole purpose is to manage the robot's overall state and direct the other nodes.

### Step-by-Step State Logic:

1.  **`IDLE` State**: The robot is waiting for a command. The `Orchestrator` is subscribed to the `/voice_command` topic.
2.  **Command Received**: When a string is published on `/voice_command`, the `Orchestrator` transitions to the **`PLANNING`** state. It takes the text and sends it to the `/llm_planner_node` (or has the planner as an internal component).
3.  **Plan Received**: The `Orchestrator` subscribes to the `/robot_plan` topic. When it receives a valid JSON plan, it transitions to the **`EXECUTING`** state.
4.  **Execution**: The `Orchestrator` iterates through the plan's steps. For each step, it calls the appropriate ROS 2 action server (e.g., `NavigateToPose` for a `go_to` action).
5.  **Completion/Failure**: If the entire plan completes successfully, the `Orchestrator` returns to the `IDLE` state. If any action fails, it enters an **`ERROR`** state, where it might ask the user for clarification or signal for help.

### Code Example 1: The `Orchestrator` Node Skeleton

This code shows the structure of the main control node, including its state management and ROS 2 communication points.

```python
# file: orchestrator_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import json
from enum import Enum

# Define the states of our robot
class RobotState(Enum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    ERROR = 3

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        
        # --- 1. State Management ---
        # The robot starts in the IDLE state.
        self.state = RobotState.IDLE
        self.get_logger().info("Orchestrator is in IDLE state.")
        
        # --- 2. ROS 2 Subscribers ---
        # Subscribes to the final transcribed voice command.
        self.voice_command_sub = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10)
            
        # Subscribes to the JSON plan generated by the LLM.
        self.plan_sub = self.create_subscription(
            String, 'robot_plan', self.plan_callback, 10)
            
        # --- 3. ROS 2 Action Clients ---
        # An action client for Nav2. This is used to send navigation goals.
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- 4. Plan Storage ---
        # A queue to hold the steps of the current plan.
        self.plan_queue = []

        # Start a timer to periodically check for state transitions
        self.execution_timer = self.create_timer(1.0, self.execution_loop)

    def voice_command_callback(self, msg):
        """Callback for when a voice command is received."""
        # Only process commands if we are in the IDLE state.
        if self.state == RobotState.IDLE:
            self.get_logger().info(f'Voice command received: "{msg.data}". Transitioning to PLANNING.')
            self.state = RobotState.PLANNING
            # In a real system, you'd publish this to the LLM planner.
            # For this example, we assume the planner is listening.
        else:
            self.get_logger().warn("Ignoring voice command while not in IDLE state.")

    def plan_callback(self, msg):
        """Callback for when a new plan is received from the LLM planner."""
        if self.state == RobotState.PLANNING:
            self.get_logger().info("Received a new plan. Transitioning to EXECUTING.")
            try:
                # The plan is a JSON list of dictionaries.
                self.plan_queue = json.loads(msg.data)
                self.state = RobotState.EXECUTING
            except json.JSONDecodeError:
                self.get_logger().error("Failed to parse plan. Returning to IDLE.")
                self.state = RobotState.IDLE
        else:
            self.get_logger().warn("Ignoring plan while not in PLANNING state.")
            
    def execution_loop(self):
        """
        The main loop that executes the plan, step by step.
        This is run by a timer to be non-blocking.
        """
        if self.state == RobotState.EXECUTING:
            if not self.plan_queue:
                # If the plan queue is empty, the plan is complete.
                self.get_logger().info("Plan execution complete. Returning to IDLE.")
                self.state = RobotState.IDLE
                return
            
            # Dequeue the next action.
            next_action = self.plan_queue.pop(0)
            action_type = next_action.get("action")
            parameter = next_action.get("parameter")
            
            self.get_logger().info(f"Executing action: {action_type} with parameter: {parameter}")
            
            # --- This is where the Plan Executor logic goes ---
            self.execute_action(action_type, parameter)

    def execute_action(self, action_type, parameter):
        # Placeholder for the plan executor logic
        self.get_logger().info("Plan Executor would handle this action...")
        # See Code Example 2 for the implementation of this part.


def main(args=None):
    rclpy.init(args=args)
    orchestrator_node = OrchestratorNode()
    rclpy.spin(orchestrator_node)
    orchestrator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## The Plan Executor: Translating Plan to Action

The `Orchestrator` manages the *what* and *when*; the **Plan Executor** handles the *how*. This component (which can be a class or a set of functions within the `Orchestrator`) is responsible for taking a single step from the plan and making the correct ROS 2 action call.

### Code Example 2: A Python `PlanExecutor` Class

This class could be instantiated inside your `OrchestratorNode` to handle the action calls.

```python
# file: plan_executor.py
# This code would be integrated into the OrchestratorNode class.

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
# Import other custom action types, e.g., for manipulation
# from my_robot_interfaces.action import PickAndPlace

class PlanExecutor:
    def __init__(self, node: Node):
        """
        Initializes the PlanExecutor.
        Args:
            node (Node): The parent ROS 2 node (the Orchestrator).
        """
        self._node = node
        self._logger = node.get_logger()
        
        # --- 1. Action Clients ---
        # Client for Nav2's navigation action.
        self._nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        # Client for a hypothetical manipulation action.
        # self._manipulation_client = ActionClient(node, PickAndPlace, 'pick_and_place')
        
        # --- 2. Location Dictionary ---
        # In a real robot, this would be managed by a map or perception system.
        self.location_presets = {
            "table": PoseStamped(...), # Pre-filled PoseStamped message
            "fridge": PoseStamped(...),
            "charging_station": PoseStamped(...)
        }

    def execute_action(self, action_type: str, parameter: str):
        """
        Executes a single action from the plan.
        """
        if action_type == "go_to":
            self.execute_go_to(parameter)
        elif action_type == "pick_up":
            self.execute_pick_up(parameter)
        # Add other actions here...
        else:
            self._logger.error(f"Unknown action type: {action_type}")
            # Here you would handle the error, maybe stop the plan.

    def execute_go_to(self, location_name: str):
        """
        Handles the 'go_to' action by calling the Nav2 action server.
        """
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._logger.error("Nav2 action server not available!")
            return

        # --- 3. Create the Goal Message ---
        # Look up the pose from our preset dictionary.
        target_pose = self.location_presets.get(location_name)
        if not target_pose:
            self._logger.error(f"Unknown location: {location_name}")
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        self._logger.info(f"Sending navigation goal to '{location_name}'...")
        
        # --- 4. Send the Goal Asynchronously ---
        # We send the goal and register a callback for when it's done.
        # This prevents our main node from blocking.
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)
            
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Callback for when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.error('Navigation goal rejected!')
            return
        self._logger.info('Navigation goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Callback for the final result of the navigation action."""
        result = future.result().result
        self._logger.info(f"Navigation result: {result.code}")
        # Here, you would signal back to the Orchestrator that this step is complete
        # so it can proceed with the next action in the queue.

    def nav_feedback_callback(self, feedback_msg):
        """Callback for receiving feedback during navigation."""
        feedback = feedback_msg.feedback
        # self._logger.info(f"Distance to goal: {feedback.distance_remaining:.2f}m")
```

> **Best Practice: Asynchronous Actions**
> ROS 2 Actions are non-blocking. When you send a goal, you should use `send_goal_async` and attach callback functions (`add_done_callback`) to handle the response. This allows your main node to remain responsive to other events (like a stop command) while the action is executing in the background. The `Orchestrator` should not proceed to the next plan step until the "result" callback for the current action has signaled success.

---

## Putting It All Together: A Complete Run-Through

Let's trace the "get me the soda can" command through our complete system.

<!-- ![Figure 14-2: Sequence Diagram for a Voice Command](img/capstone_sequence_diagram.png) -->
*This sequence diagram shows the chronological flow of messages. The User's voice command triggers the Whisper node, which sends text to the Orchestrator. The Orchestrator initiates planning with the LLM node. Once a plan is received, the Orchestrator sends a series of goals to the Nav2 and Manipulation action servers, waiting for a 'success' result from each before continuing.*

1.  **Voice**: User says, "get me the soda can".
2.  **`speech_to_text_publisher`**: Captures audio, transcribes it, and publishes `"get me the soda can"` to the `/voice_command` topic.
3.  **`OrchestratorNode`**: Is `IDLE`. Receives the command, transitions to `PLANNING`. It forwards the command to the `llm_planner_node`.
4.  **`llm_planner_node`**: Receives the text. It builds a prompt including the robot's action space and world state. It calls the LLM API.
5.  **LLM**: Returns a JSON plan string: `[{"action": "go_to", "parameter": "fridge"}, {"action": "pick_up", "parameter": "soda_can"}, ...]`
6.  **`llm_planner_node`**: Validates the JSON and publishes it to the `/robot_plan` topic.
7.  **`OrchestratorNode`**: Is `PLANNING`. Receives the plan, transitions to `EXECUTING`, and loads the steps into its `plan_queue`.
8.  **`OrchestratorNode` (Execution Loop)**:
    *   Dequeues the first step: `{"action": "go_to", "parameter": "fridge"}`.
    *   Calls its internal `PlanExecutor`'s `execute_action` method.
    *   `PlanExecutor` looks up the pose for "fridge" and sends a goal to the `NavigateToPose` action server (Nav2).
    *   The node now waits. The action is executing in the background.
9.  **Nav2**: Receives the goal and begins navigating the robot to the fridge, publishing velocity commands.
10. **`OrchestratorNode` (Result Callback)**: After some time, Nav2 succeeds. The `nav_result_callback` is triggered. Inside this callback, a signal is given to the `Orchestrator` to run its `execution_loop` again.
11. **`OrchestratorNode` (Execution Loop)**:
    *   Dequeues the next step: `{"action": "pick_up", "parameter": "soda_can"}`.
    *   Calls the `PlanExecutor`, which in turn calls the manipulation action server.
    *   ...and so on, until the plan queue is empty.
12. **`OrchestratorNode`**: The queue is empty. The `execution_loop` transitions the state back to `IDLE`, ready for the next command.

---

### Summary

This capstone project successfully integrated every major concept from this book into a single, cohesive, autonomous system. We designed a high-level **Orchestrator node** to act as a state machine, managing the flow from voice input to task execution. We implemented a **Plan Executor** to translate the abstract steps from an LLM's plan into concrete ROS 2 action calls to servers like Nav2. By tracing a command from start to finish, we have demonstrated a complete and modern robotics software architecture. While this is a conceptual blueprint, it provides a powerful and scalable foundation for building truly intelligent and interactive humanoid robots capable of understanding and acting upon human intent. Congratulations on completing this journey into the future of robotics.
