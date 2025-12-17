---
title: 'Chapter 12: Behavior Trees for Robotics'
metadata:
  chapter_number: 12
  keywords: ['behavior trees', 'bt', 'robot behavior', 'finite state machine', 'fsm', 'py_trees']
---

# Chapter 12: Behavior Trees for Robotics


## Goals


- **To introduce Behavior Trees (BTs)** as a powerful, scalable, and modular framework for creating complex robot behaviors.
- **To explain the different types of nodes** in a BT (Sequence, Fallback, Action, Condition) and their specific roles.
- **To demonstrate how to build complex, intelligent behaviors** by composing simple, reusable nodes into a hierarchical tree structure.

## Learning Outcomes


Upon completing this chapter, you will be able to:

- **Explain the advantages of Behavior Trees** over traditional Finite State Machines (FSMs), particularly for complex tasks.
- **Describe the function of the four primary BT node types** and how they control the flow of execution.
- **Design a simple Behavior Tree** for a common robotics task, like a "fetch" or "patrol" operation.

## Full Explanations of Concepts



### Why Not Finite State Machines (FSMs)?

For simple tasks, FSMs are a good tool. However, as a robot's required behaviors become more complex, FSMs begin to break down.

- **State-Space Explosion:** The primary weakness of FSMs is that the number of transitions between states can grow exponentially. If you have 10 states, and you decide to add an 11th state (e.g., a new "error-handling" state), you might have to create and manage transitions from all 10 existing states to this new one, and from the new one back to all the others. This becomes a tangled "spaghetti" mess that is very difficult to manage and debug.
- **Lack of Modularity:** An FSM represents a single, monolithic logic block. It's very difficult to take a "sub-FSM" and reuse it in a different context. They are not inherently modular or composable.

**Behavior Trees** were developed in the video game industry to solve exactly this problem: how to model complex, non-player character (NPC) AI in a way that is scalable, reusable, and easy for designers to understand.

### Introduction to Behavior Trees

**Definition:** A Behavior Tree is a directed acyclic graph, organized as a tree, that controls the flow of decision-making for an autonomous agent.

**How it Works:**
Unlike an FSM, a BT is not event-driven. Instead, it is "ticked" at a regular frequency (e.g., 10 times per second).
1. A "tick" signal originates at the **root** of the tree.
2. The tick propagates down the tree, activating a subset of nodes according to specific rules.
3. Each node, when ticked, performs its function and returns one of three statuses to its parent:
    - `SUCCESS`: The node has successfully completed its purpose.
    - `FAILURE`: The node has failed to achieve its purpose.
    - `RUNNING`: The node has not yet finished its task and needs to be ticked again in the future.

This `SUCCESS`/`FAILURE`/`RUNNING` system is the core mechanic that governs the entire tree's behavior.

**Advantages of BTs:**
- **Modular:** Each node is a self-contained unit of functionality. A `Navigate` action can be reused in many different parts of the tree.
- **Hierarchical:** The tree structure naturally represents the hierarchical nature of tasks (a high-level task is composed of smaller sub-tasks).
- **Reactive:** Because the tree is ticked frequently, it can react quickly to changes in the world.
- **Readable:** A graphical representation of a BT is far more intuitive to read and debug than a complex FSM diagram.

### Core Node Types

There are two main categories of nodes: **Control Flow Nodes** (which have children) and **Execution Nodes** (which are the leaves of the tree).

#### Control Flow Nodes (Branches)

1.  **Sequence (`→`)**
    - **Purpose:** To execute a sequence of tasks in a specific order.
    - **Logic:** Ticks its children one by one from left to right.
        - If a child returns `FAILURE` or `RUNNING`, the Sequence node stops immediately and returns that same status to its parent. It will *not* execute the rest of its children.
        - It only moves to the next child if the current one returns `SUCCESS`.
        - If all children return `SUCCESS`, the Sequence itself returns `SUCCESS`.
    - **Analogy:** An **AND** gate. All children must succeed for the sequence to succeed.

2.  **Fallback (`?`)** (also called a **Selector**)
    - **Purpose:** To try a series of tasks in order of priority until one succeeds.
    - **Logic:** Ticks its children one by one from left to right.
        - If a child returns `SUCCESS` or `RUNNING`, the Fallback node stops immediately and returns that same status to its parent. It has found a working solution.
        - It only moves to the next child if the current one returns `FAILURE`.
        - If all children return `FAILURE`, the Fallback itself returns `FAILURE`.
    - **Analogy:** An **OR** gate or a prioritized `try-catch` block.

#### Execution Nodes (Leaves)

These are the nodes that do the actual work.

1.  **Action**
    - **Purpose:** To perform an action that affects the world or the robot.
    - **Examples:** `MoveArmToPose`, `CalculatePath`, `OpenGripper`, `PlaySound`.
    - **Logic:** When ticked, it initiates its action. If the action is instantaneous, it can return `SUCCESS` or `FAILURE` on the same tick. If the action takes time (like navigation), it will return `RUNNING` on the first tick and continue to return `RUNNING` on subsequent ticks until the action is complete, at which point it will return `SUCCESS` or `FAILURE`.

2.  **Condition**
    - **Purpose:** To check a state of the robot or the world.
    - **Examples:** `IsBatteryLow?`, `IsObjectDetected?`, `IsGripperOpen?`.
    - **Logic:** A condition node is a check, not an action. It never returns `RUNNING`. When ticked, it immediately returns `SUCCESS` if the condition is true or `FAILURE` if the condition is false.

### The Blackboard: A Shared Brain

How do different nodes in a big tree share information? For example, how does the `NavigateTo` action know the location of the object found by the `DetectObject` action? They use a **Blackboard**.

- **Definition:** The blackboard is a centralized, key-value data store that is accessible to all nodes in the tree. It acts as the tree's short-term memory.
- **Example:**
    1. A `DetectObject` node runs and finds a soda can. It writes to the blackboard: `blackboard.set("target_pose", <pose_of_can>)`.
    1. A `DetectObject` node runs and finds a soda can. It writes to the blackboard: `blackboard.set(