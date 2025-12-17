---
id: 1
title: Establish Project Constitution and Principles
stage: constitution
date: 2025-12-06
surface: agent
model: gemini
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution,principles]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
  none
---

## Prompt

/sp.constitution /sp.constitution
Project: Physical AI and Humanoid Robotics Textbook System

Purpose: 
Develop a fully AI-driven, interactive textbook platform that comprehensively covers the following topics:
- Foundations of Physical AI
- Robotics Fundamentals
- Sensors and Actuators
- ROS 2 Essentials
- Gazebo Simulation
- NVIDIA IsaacSim
- Perception and Vision
- Vision-Language-Action (VLA) Models
- Machine Learning for Robotics
- Humanoid Robotics
- Jetson Edge Deployment
- AI Agents for Robotics
- Hands-on Labs
- Quizzes and Assessments
- Urdu Translation Layer
- Personalization Layer

Simulation System Goals:
1. Auto-generate a complete textbook with all listed topics.
2. Integrate a REC chatbot link into each chapter for interactive assistance.
3. Enable users to add selected text and annotate chapters.
4. Provide AI-powered interactive features within chapters (Q&A, explanations, examples).
5. Adjust chapter difficulty dynamically based on user performance and preferences.
6. Offer Urdu translation for all textbook content.
7. Support text tagging and system modules: Frontend: Docasaurus, Backend: FastAPI, VectorDB: Qdrant, DB: Neon, Postgres, Auth: Better auth, LLMs: Cloud, Gemini, OPD.

Standards and Requirements:
- All content and responses must strictly follow the chapter content.
- Architecture must be modular, clean, and typed with proper code structure.
- Book content must follow a structured, educational design with theory, examples, exercises, and assessments.
- Every API endpoint and interactive feature must be documented, testable, and reproducible.
- The system must support AI-driven personalization, translation, and hands-on labs seamlessly.

## Response snapshot

Updated the project constitution in `.specify/memory/constitution.md` with version 1.0.0, establishing five core principles: Content Fidelity, Modular and Clean Architecture, Structured Educational Design, Testability and Reproducibility, and Seamless AI Integration. Also updated the `.specify/templates/plan-template.md` to reflect these principles.

## Outcome

- ✅ Impact: High. Established the foundational principles for the project.
- 🧪 Tests: None.
- 📁 Files: 2 files modified.
- 🔁 Next prompts: User can now proceed with other commands like `/sp.plan` or `/sp.specify`.
- 🧠 Reflection: The process of creating a constitution from a project description helps to solidify the project's core values and standards early on.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): n/a
