# Technical Specification: Cohere-Powered RAG Chatbot

**Version:** 1.0
**Status:** Draft
**Author:** Gemini Agent

## 1. System Overview

This document outlines the technical specification for a Retrieval-Augmented Generation (RAG) chatbot integrated into a book-publishing website. The system is designed to answer user queries based exclusively on the book's content, leveraging the Cohere platform for all language model operations and Qdrant for vector storage.

The architecture is built on a FastAPI backend, follows an agent-based design pattern for modularity, and uses Neon Serverless Postgres for relational data persistence.

## 2. Core Constraints & Principles

- **Model Provider:** All LLM interactions (embeddings, generation) **MUST** use the Cohere API. OpenAI services are strictly prohibited.
- **Knowledge Scope:** The system is a closed-book system. It **MUST NOT** use any information outside of the provided book content or user-selected text.
- **Answer Grounding:** If the answer to a query cannot be found in the provided context, the system **MUST** respond with the exact message: "The answer is not present in the provided content."
- **Correctness over Verbosity:** The system should provide concise, accurate answers and cite sources (chapter/section) whenever possible.
- **Production-Ready Code:** All code must be clean, well-documented, and include error handling.

## 3. Technology Stack

- **Backend Framework:** FastAPI
- **LLM & Embeddings Provider:** Cohere
- **Vector Database:** Qdrant Cloud (Free Tier)
- **Relational Database:** Neon Serverless Postgres
- **Schema Validation:** Pydantic
- **ORM:** SQLAlchemy (Async)

## 4. System Architecture

The system uses a modular, agent-based architecture to separate concerns.

![System Architecture Diagram](https://placehold.co/800x400?text=Architecture+Diagram)
*(Placeholder for a diagram showing User -> FastAPI -> Agents -> {Qdrant, Cohere, Neon})*

### 4.1. Agents

#### 4.1.1. Retrieval Agent
- **Responsibility:** Fetches relevant context from the vector database.
- **Logic:**
    1. Receives a user query string.
    2. Uses the Cohere `embed-english-v3.0` model to generate a vector embedding for the query.
    3. Queries the Qdrant collection to find the top-k most similar document chunks based on cosine similarity.
    4. Returns the retrieved document chunks with their metadata.
- **Component:** `backend/src/agents/retrieval_agent.py`

#### 4.1.2. Generation Agent
- **Responsibility:** Generates a user-facing answer based on the query and provided context.
- **Logic:**
    1. Receives the user query and a list of context documents (either from the Retrieval Agent or from user selection).
    2. Constructs a precise prompt for the Cohere Chat API (`command-r` or `command-r-plus` model). The prompt will include the context, the user's question, and strict instructions to only use the provided context.
    3. Uses Cohere's `Rerank` API before generation to improve the quality of the retrieved documents.
    4. If no context is provided or the reranked documents have low relevance scores, it returns the standard "not found" message.
    5. Sends the request to the Cohere API.
    6. Processes the response, extracting the answer and source citations.
- **Component:** `backend/src/agents/generation_agent.py`

#### 4.1.3. Context Validation Agent
- **Responsibility:** Enforces the context scope rules. This is less a standalone agent and more a collection of validation logic within the API endpoints.
- **Logic:**
    - For `/chat/book` mode: Validates that the context comes from the Retrieval Agent.
    - For `/chat/selection` mode: Explicitly blocks any calls to the Retrieval Agent and ensures only the user-provided text is passed to the Generation Agent.

## 5. Data Models & Schemas

### 5.1. Qdrant Vector Store

- **Collection Name:** `humanoid_robot_book`
- **Vector Parameters:**
    - `size`: 1024 (for Cohere `embed-english-v3.0`)
    - `distance`: Cosine
- **Payload Schema (Metadata):**
    ```json
    {
      "text_chunk": "The actual text content of the chunk.",
      "chapter": "Chapter 1",
      "section": "1.1 Introduction to ROS",
      "source_path": "docs/Module1-The robotic-nervous-system-ros-2/ros-2-nodes-topics-services.md"
    }
    ```

### 5.2. Neon Postgres (SQLAlchemy Models)

- **`user_sessions` Table:**
    ```sql
    CREATE TABLE user_sessions (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        created_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),
        metadata JSONB
    );
    ```

- **`chat_history` Table:**
    ```sql
    CREATE TABLE chat_history (
        id BIGSERIAL PRIMARY KEY,
        session_id UUID REFERENCES user_sessions(id) ON DELETE CASCADE,
        timestamp TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),
        role VARCHAR(20) NOT NULL, -- 'user' or 'assistant'
        content TEXT NOT NULL,
        mode VARCHAR(20) NOT NULL, -- 'book' or 'selection'
        trace_id UUID -- Foreign key to trace_logs
    );
    ```

- **`trace_logs` Table:** (For observability)
    ```sql
    CREATE TABLE trace_logs (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        request_id UUID,
        timestamp TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),
        prompt TEXT,
        retrieved_context JSONB,
        reranked_context JSONB,
        final_answer TEXT,
        llm_response JSONB
    );
    ```

## 6. API Endpoints (FastAPI)

All endpoints will be defined in `backend/src/api/endpoints.py`.

### 6.1. `POST /ingest`

- **Purpose:** Ingests and processes the book content into the vector store.
- **Request Body:**
    ```json
    {
      "source_path": "path/to/book/markdown/files"
    }
    ```
- **Process:**
    1. Reads Markdown files from the specified `source_path`.
    2. Chunks the text into meaningful segments (e.g., by paragraph or section).
    3. Extracts metadata (chapter, section) from the file structure or content.
    4. For each chunk:
        a. Generates an embedding using the Cohere API.
        b. Upserts the vector and its payload (chunk, metadata) into the Qdrant collection.
- **Response:**
    - `202 Accepted`: On successful start of the background task.
    - `400 Bad Request`: For invalid input.

### 6.2. `POST /chat/book`

- **Purpose:** Handles a user query using the entire book as context (RAG).
- **Request Body (Pydantic Model):**
    ```python
    class BookChatRequest(BaseModel):
        session_id: UUID
        query: str
    ```
- **Process:**
    1. Log the incoming request.
    2. Use the **Retrieval Agent** to get relevant document chunks from Qdrant.
    3. Use the **Generation Agent** to create a response, reranking the retrieved chunks first.
    4. Record the user query and the assistant's response in the `chat_history` table.
    5. Record the full RAG trace in the `trace_logs` table.
- **Response:**
    ```json
    {
      "answer": "The generated response from the assistant.",
      "sources": [
        {"chapter": "Chapter 1", "section": "1.1 Introduction to ROS"}
      ],
      "session_id": "...",
      "trace_id": "..."
    }
    ```

### 6.3. `POST /chat/selection`

- **Purpose:** Handles a user query using only user-selected text as context.
- **Request Body (Pydantic Model):**
    ```python
    class SelectionChatRequest(BaseModel):
        session_id: UUID
        query: str
        selected_text: str
    ```
- **Process:**
    1. **CRITICAL:** The **Retrieval Agent is bypassed**.
    2. The `selected_text` is treated as the sole context.
    3. Use the **Generation Agent** to create a response. The prompt will be structured to heavily emphasize that only the `selected_text` can be used.
    4. If the `query` cannot be answered from the `selected_text`, the agent returns the standard "not found" message.
    5. Record the interaction in `chat_history` and `trace_logs`.
- **Response:**
    ```json
    {
      "answer": "The generated response from the assistant.",
      "sources": [], // Sources are not applicable here
      "session_id": "...",
      "trace_id": "..."
    }
    ```

## 7. Data Ingestion and Chunking

- **Source Format:** Markdown (`.md`) files.
- **Chunking Strategy:**
    1. **Recursive Character Text Splitter:** Initially split by section headers (`##`, `###`), then by paragraphs (`\n\n`).
    2. **Chunk Size:** Aim for chunks of ~300-500 tokens.
    3. **Overlap:** Use a small chunk overlap (~50 tokens) to maintain contextual coherence between chunks.
- **Metadata Extraction:** The chapter and section titles will be inferred from the directory structure and Markdown headers of the source files. This is critical for citation.

## 8. Configuration and Environment

- All sensitive keys and configuration variables (Cohere API key, Qdrant URL, Neon DB connection string) will be managed via a `.env` file and loaded using Pydantic's `BaseSettings`.
- **File:** `backend/.env`
- **Schema:** `backend/src/core/config.py`

```json
COHERE_API_KEY="sk-..."
QDRANT_URL="https://..."
QDRANT_API_KEY="..."
NEON_DATABASE_URL="postgresql+asyncpg://..."
```