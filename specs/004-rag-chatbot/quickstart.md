# Quickstart Guide: RAG Chatbot

This guide provides the basic steps to set up the local environment and run the RAG chatbot backend.

## 1. Prerequisites

-   Python 3.9+
-   Poetry for dependency management
-   Access to Cohere, Qdrant, and Neon cloud services.

## 2. Environment Setup

1.  **Clone the repository** (if you haven't already).
2.  **Navigate to the `backend` directory**.
3.  **Create a `.env` file** and populate it with the following credentials. Do not commit this file.

    ```bash
    # .env

    # Cohere API Key
    COHERE_API_KEY="<your_cohere_api_key>"

    # Qdrant Cloud URL and API Key
    QDRANT_API_URL="<your_qdrant_cloud_url>"
    QDRANT_API_KEY="<your_qdrant_api_key>" # If applicable

    # Neon Serverless Postgres Connection String
    NEON_DATABASE_URL="<your_neon_database_url>"
    ```

4.  **Install dependencies** using Poetry:
    ```bash
    poetry install
    ```

## 3. Running the Application

1.  **Start the FastAPI server**:
    ```bash
    poetry run uvicorn main:app --reload
    ```
    The API will be available at `http://127.0.0.1:8000`.

2.  **Trigger the Ingestion Process**:
    Send a `POST` request to the `/ingest` endpoint to process and vectorize the book content.
    ```bash
    curl -X POST http://127.0.0.1:8000/ingest
    ```

3.  **Interact with the Chat API**:
    Once ingestion is complete, you can send `POST` requests to the `/chat/book` or `/chat/selection` endpoints as defined in the `contracts/api.yaml` file.

    **Example (Book-wide mode)**:
    ```bash
    curl -X POST http://127.0.0.1:8000/chat/book \
    -H "Content-Type: application/json" \
    -d 
      "{
        \"query\": \"How does the middleware for robot control work?\"
      }"
    ```

