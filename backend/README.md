# Humanoid Robot Book - Backend Service

This directory contains the FastAPI backend for the RAG (Retrieval-Augmented Generation) chatbot.

## Features

- **Cohere-Powered:** Uses Cohere for embeddings, reranking, and chat generation.
- **Qdrant Vector Store:** Stores book content embeddings for fast semantic search.
- **Neon Postgres DB:** Stores all chat history and RAG traces for observability.
- **Dual RAG Modes:**
    - `/chat/book`: Answers questions based on the entire book.
    - `/chat/selection`: Answers questions based only on user-selected text.
- **Asynchronous Ingestion:** A `/ingest` endpoint processes and embeds book content in the background.

## Setup and Installation

1.  **Install Dependencies:**
    Navigate to the `backend` directory and install the required Python packages.
    ```bash
    pip install -r requirements.txt
    ```

2.  **Set Up Environment Variables:**
    Create a file named `.env` in the `backend` directory by copying the example below.

    ```env
    # .env file
    COHERE_API_KEY="your_cohere_api_key_here"
    QDRANT_URL="http://localhost:6333" # Or your Qdrant Cloud URL
    QDRANT_API_KEY="your_qdrant_api_key_if_any"
    NEON_DATABASE_URL="postgresql+asyncpg://user:password@host:port/dbname"
    ```
    - `COHERE_API_KEY`: Your API key from the Cohere dashboard.
    - `QDRANT_URL`: The URL for your Qdrant instance. For local development, this can be a local file path (e.g., `./local_qdrant`). For production, use your Qdrant Cloud URL.
    - `NEON_DATABASE_URL`: Your connection string from the Neon dashboard. It must be an `asyncpg` compatible URL.

## Running the Application

1.  **Start the FastAPI Server:**
    Use `uvicorn` to run the application. From the root of the project directory, run:
    ```bash
    uvicorn backend.main:app --reload
    ```
    The `--reload` flag is for development and automatically reloads the server on code changes.

2.  **Access the API Docs:**
    Once the server is running, you can access the interactive API documentation (Swagger UI) at `http://127.0.0.1:8000/docs`.

## Usage Workflow

1.  **Initialize the Vector Database:**
    Before you can chat, you must ingest the book content. Make a `POST` request to the `/api/v1/ingest` endpoint. You can do this easily from the `/docs` page.

    The default `source_path` is `frontend/static/docs`, which contains the markdown files for the book.

2.  **Chat with the Book:**
    Make `POST` requests to the `/api/v1/chat/book` endpoint with your query. You will need to provide a `session_id` (this can be any valid UUID).

3.  **Chat with a Selection:**
    Make `POST` requests to the `/api/v1/chat/selection` endpoint. In addition to the `session_id` and `query`, you must provide the `selected_text`.
