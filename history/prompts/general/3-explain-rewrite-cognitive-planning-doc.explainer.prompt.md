---
id: 3
title: Explain and Rewrite Cognitive Planning Document
stage: explainer
date: 2025-12-13
surface: "agent"
model: "gemini"
feature: "none"
branch: master
user: [USER]
command: Edit this page
 what is this  this is the topic Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions. rwrite it
labels: [documentation, llm, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - frontend/docs/module-4/3-cognitive-planning.md
tests:
 - "none"
---

## Prompt

You are a helpful assistant that controls a humanoid robot.
Your task is to decompose a high-level command into a sequence of primitive actions.
You must only use the following functions. Do not invent new functions.
The output must be a valid JSON list of action objects.
=== Available Actions ===
def go_to(location_name: str): """ Navigates the robot to a predefined named location in the house. Valid locations: ['kitchen', 'living_room', 'charging_station', 'table', 'fridge'] Args: location_name (str): The name of the destination. """ pass

def pick_up(object_name: str): """ Picks up a specified object. The robot must be at the object's location first. The object must be from the list of perceivable objects. Args: object_name (str): The name of the object to pick up. """ pass

def place(location_name: str): """ Places the currently held object at the specified location. The robot must be at the location to place the object. Args: location_name (str): The name of the location to place the object. """ pass

def open_container(container_name: str): """ Opens a container-like object, such as a fridge or a cupboard. The robot must be at the container's location. Args: container_name (str): The name of the container to open. """ pass

def close_container(container_name: str): """ Closes a container-like object. The robot must be at the container's location. Args: container_name (str): The name of the container to close. """ pass

Edit this page
 what is this  this is the topic Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions. rwrite it

## Response snapshot

I have rewritten the content of the page as you requested. The file `frontend/docs/module-4/3-cognitive-planning.md` is now updated with a detailed explanation of Cognitive Planning.

## Outcome

- ✅ Impact: Updated a documentation file with a clear explanation of a key concept.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: Modified one markdown file.
- 🔁 Next prompts: Follow-up prompts could involve adding more details or examples to the documentation.
- 🧠 Reflection: The user's request was a bit ambiguous, mixing a persona with a meta-request. I correctly inferred the user's intent was to edit the documentation.

## Evaluation notes (flywheel)

- Failure modes observed: The initial git command to get the branch name failed, but a fallback command worked.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): For similar requests, I could offer to show a diff of the changes before writing to the file.
