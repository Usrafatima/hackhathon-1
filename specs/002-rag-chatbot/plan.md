# Implementation Plan: Cohere-Powered RAG Chatbot

**Version:** 1.0
**Status:** Ready for Implementation
**Reference Spec:** `spec.md`

## Phase 1: Backend Setup & Configuration

-   **Goal:** Initialize the FastAPI application and configure it to connect to external services (Cohere, Qdrant, Neon).
-   **Tasks:**
    1.  **Initialize FastAPI Project:**
        -   [ ] Create the basic file structure in `backend/src`.
        -   [ ] Files to create: `main.py`, `api/endpoints.py`, `core/config.py`.
    2.  **Environment Configuration:**
        -   [ ] Create `backend/.env` file to store secrets.
        -   [ ] Implement a Pydantic `Settings` class in `core/config.py` to load and validate environment variables (`COHERE_API_KEY`, `QDRANT_URL`, `NEON_DATABASE_URL`).
    3.  **Database Setup (Neon):**
        -   [ ] Define SQLAlchemy ORM models in `db/models.py` for `UserSession`, `ChatHistory`, and `TraceLog`.
        -   [ ] Implement a database session manager in `db/database.py` to handle async connections.
        -   [ ] Create an initialization script or function to create the tables in the Neon database.
    4.  **Vector DB Setup (Qdrant):**
        -   [ ] Create a client utility in `db/vector_db.py` to connect to the Qdrant Cloud instance.
        -   [ ] Write a function to create the `humanoid_robot_book` collection with the correct vector parameters as defined in the spec.

## Phase 2: Data Ingestion Pipeline

-   **Goal:** Build the logic to read, chunk, embed, and store the book content.
-   **Tasks:**
    1.  **Markdown Processing:**
        -   [ ] Create a module `pipeline/process_markdown.py`.
        -   [ ] Implement a function to recursively find all `.md` files in a given directory.
        -   [ ] Implement text splitting logic using a library like `langchain.text_splitter` (configured for markdown).
        -   [ ] Implement metadata extraction logic to capture chapter/section from file paths and headers.
    2.  **Embedding and Storage:**
        -   [ ] In the same module, create a function that takes text chunks.
        -   [ ] This function will call the Cohere API to generate embeddings for the chunks in batches.
        -   [ ] It will then use the Qdrant client to `upsert` these embeddings along with their text and metadata payload.
    3.  **API Endpoint:**
        -   [ ] Implement the `POST /ingest` endpoint in `api/endpoints.py`.
        -   [ ] This endpoint should use FastAPI's `BackgroundTasks` to run the ingestion process asynchronously.

## Phase 3: Agent Implementation

-   **Goal:** Develop the core logic for the Retrieval and Generation agents.
-   **Tasks:**
    1.  **Retrieval Agent:**
        -   [ ] Create `agents/retrieval_agent.py`.
        -   [ ] Implement a `search` function that:
            -   Takes a `query: str`.
            -   Embeds the query using Cohere.
            -   Searches the Qdrant collection.
            -   Returns a list of `Document` objects.
    2.  **Generation Agent:**
        -   [ ] Create `agents/generation_agent.py`.
        -   [ ] Implement a `generate_answer` function that:
            -   Takes a `query: str` and `context: List[Document]`.
            -   Uses the Cohere `rerank` API to score and sort the context documents.
            -   Builds a detailed prompt using the reranked context and the query.
            -   Calls the Cohere `chat` API to get the final answer.
            -   Includes logic to return the "not found" message if context is insufficient.

## Phase 4: API Chat Endpoints

-   **Goal:** Expose the chatbot functionality through the public-facing API.
-   **Tasks:**
    1.  **Book-wide Chat Endpoint:**
        -   [ ] Implement the `POST /chat/book` endpoint in `api/endpoints.py`.
        -   [ ] Define the `BookChatRequest` Pydantic model.
        -   [ ] Orchestrate the flow: `request -> retrieval_agent -> generation_agent -> response`.
        -   [ ] Implement database logging for the chat history and trace.
    2.  **Selection-only Chat Endpoint:**
        -   [ ] Implement the `POST /chat/selection` endpoint.
        -   [ ] Define the `SelectionChatRequest` Pydantic model.
        -   [ ] **Crucially**, ensure the retrieval agent is *not* called.
        -   [ ] Orchestrate the flow: `request -> generation_agent (with selected_text) -> response`.
        -   [ ] Implement database logging.

## Phase 5: Testing and Validation

-   **Goal:** Ensure the system works as expected and adheres to all constraints.
-   **Tasks:**
    1.  **Unit Tests:**
        -   [ ] Write unit tests for the markdown chunking logic.
        -   [ ] Write unit tests for the agent functions (mocking API calls to Cohere and Qdrant).
    2.  **Integration Tests:**
        -   [ ] Write an integration test for the `/ingest` endpoint.
        -   [ ] Write integration tests for both `/chat/book` and `/chat/selection` endpoints, testing for:
            -   Successful answer generation.
            -   Correct "not found" responses.
            -   Source citation accuracy.
            -   Strict adherence to the `selection-only` rule.
    3.  **Manual Validation:**
        -   [ ] Perform manual queries to check the quality of the answers.
        -   [ ] Verify that all chat history and trace logs are being correctly stored in the Neon database.
