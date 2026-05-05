# Humanoid Robot Book - Full Stack AI-Native Platform

This repository contains the source code for the **Humanoid Robot Book**, a comprehensive engineering roadmap and interactive learning platform for humanoid robotics.

## 🚀 Overview

The project is split into two main components:

- **Frontend (`/frontend`)**: A high-performance [Docusaurus](https://docusaurus.io/) website providing a structured, modular curriculum. Features a professional dark-theme landing page, interactive module cards, and seamless documentation navigation.
- **Backend (`/backend`)**: An AI-native [FastAPI](https://fastapi.tiangolo.com/) service that powers the integrated RAG (Retrieval-Augmented Generation) chatbot. It utilizes **Cohere** for embeddings and LLM generation, with **Qdrant** as the vector database for grounded, context-aware answers.

## 📚 Curriculum Roadmap

The book is structured into 4 key modules:

1. **ROS 2 - The Robotic Nervous System**: Hardware abstraction and real-time communication.
2. **Digital Twins - Gazebo & Unity**: High-fidelity physics simulation and virtual testing.
3. **The AI Robot Brain**: Neural controllers and autonomous decision-making.
4. **Vision & Language**: Perception, computer vision, and multimodal AI interaction.

## 🛠️ Tech Stack

- **Frontend**: React, TypeScript, Docusaurus, CSS Modules.
- **Backend**: Python, FastAPI, SQLAlchemy, Alembic.
- **AI/ML**: Cohere API, Qdrant (Vector DB), RAG Architecture.
- **Infrastructure**: PostgreSQL (Neon Serverless), Chroma (Local Development).

## 🚦 Getting Started

### Prerequisites
- Node.js (v18+)
- Python (3.10+)
- Poetry (for backend dependency management)

### Local Development

1. **Frontend**:
   ```bash
   cd frontend
   npm install
   npm start
   ```

2. **Backend**:
   ```bash
   cd backend
   # Set up your .env file with COHERE_API_KEY, etc.
   pip install -r requirements.txt
   uvicorn main:app --reload
   ```

## 📝 License

Copyright © 2025 Humanoid Robot Book Team. All rights reserved.
