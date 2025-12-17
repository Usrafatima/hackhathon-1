# Tasks: RAG Chatbot

**Feature**: `004-rag-chatbot`
**Plan**: [plan.md](./plan.md)
**Spec**: [spec.md](./spec.md)

This document breaks down the implementation of the RAG Chatbot into actionable, dependency-ordered tasks.

## Phase 1: Project Setup

These tasks initialize the project structure and environment.

- [X] T001 Create a new FastAPI project in `backend/` using Poetry.
- [X] T002 Add FastAPI, Uvicorn, Pydantic, Cohere, Qdrant-client, and psycopg2-binary as dependencies in `backend/pyproject.toml`.
- [X] T003 Create the directory structure `backend/src/`, `backend/src/api/`, `backend/src/core/`, `backend/src/services/`, `backend/src/models/`, `backend/src/db/`.
- [X] T004 Create a `.env` file in `backend/` and add placeholder variables for `COHERE_API_KEY`, `QDRANT_API_URL`, and `NEON_DATABASE_URL` based on `quickstart.md`.
- [X] T005 Implement configuration loading from the `.env` file in `backend/src/core/config.py`.

## Phase 2: Foundational Components

These are core components required by all user stories.

- [X] T006 Define Pydantic schemas for API requests and responses in `backend/src/models/schemas.py` based on `contracts/api.yaml`.
- [X] T007 Implement a database connection manager for Neon Postgres in `backend/src/db/database.py`.
- [X] T008 Implement a client connection manager for Qdrant in `backend/src/db/vector_db.py`.
- [X] T009 Define the SQLAlchemy models for `chat_sessions` and `chat_messages` in `backend/src/db/models.py` based on `data-model.md`.
- [X] T010 [P] Set up Alembic for database migrations in `backend/alembic/` and generate the initial migration for the chat history tables.
- [X] T011 Define abstract base classes for the agents (`RetrievalAgent`, `ContextValidationAgent`, `AnswerGenerationAgent`) in `backend/src/services/agents/base.py`.

## Phase 3: User Story 1 - Book-Wide Chat

**Goal**: Implement the core RAG pipeline for answering questions based on the entire book.
**Independent Test**: Can be tested via the `/chat/book` endpoint.

- [X] T012 [US1] Implement the `RetrievalAgent` in `backend/src/services/agents/retrieval_agent.py` to embed a query and fetch `k=5` chunks from Qdrant.
- [X] T013 [US1] Implement the `ContextValidationAgent` in `backend/src/services/agents/validation_agent.py` for the book-wide mode, including the guardrail to check for sufficient context.
- [X] T014 [US1] Implement the `AnswerGenerationAgent` in `backend/src/services/agents/generation_agent.py` to construct prompts and generate answers using the Cohere API.
- [X] T015 [US1] Create the `RAGOrchestrator` service in `backend/src/services/rag_service.py` to manage the flow between the agents for the book-wide mode.
- [X] T016 [US1] Implement the `/chat/book` endpoint in `backend/src/api/endpoints.py` that uses the `RAGOrchestrator`.
- [X] T017 [US1] Implement the database logging logic within the `/chat/book` endpoint to save the full interaction to the `chat_messages` table in `backend/src/api/endpoints.py`.

## Phase 4: User Story 2 - Selection-Only Chat

**Goal**: Implement the RAG pipeline for selection-only mode, ensuring vector retrieval is disabled.
**Independent Test**: Can be tested via the `/chat/selection` endpoint.

- [X] T018 [US2] Update the `ContextValidationAgent` in `backend/src/services/agents/validation_agent.py` to handle the `selection-only` mode logic.
- [X] T019 [US2] Update the `RAGOrchestrator` in `backend/src/services/rag_service.py` to accept `selected_text` and bypass the `RetrievalAgent` for selection-only mode.
- [X] T020 [US2] Implement the `/chat/selection` endpoint in `backend/src/api/endpoints.py`.
- [X] T021 [US2] Implement the database logging logic for the `/chat/selection` endpoint in `backend/src/api/endpoints.py`.

## Phase 5: Data Ingestion

**Goal**: Implement the data ingestion pipeline to populate the vector database.
**Independent Test**: Can be tested by running the `/ingest` endpoint and verifying the Qdrant collection is populated.

- [X] T022 Implement the text chunking logic in `backend/src/services/ingestion/chunking.py` based on `research.md`.
- [X] T023 Implement an embedding service in `backend/src/services/ingestion/embedding.py` to generate embeddings with Cohere.
- [X] T024 Implement the data upload logic in `backend/src/services/ingestion/load.py` to batch-upload vectors to Qdrant.
- [X] T025 Implement the `/ingest` endpoint in `backend/src/api/endpoints.py` to run the ingestion pipeline as a background task.

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T026 [P] Implement the generic error handling middleware in `backend/src/api/middleware.py` to catch exceptions and return the standard error response.
- [X] T027 [P] Create a `Dockerfile` for the `backend` service.
- [X] T028 [P] Write comprehensive unit tests for all services and agents in the `backend/tests/unit/` directory.
- [X] T029 [P] Write integration tests for the API endpoints in the `backend/tests/integration/` directory.

## Dependency Graph

-   **User Story 1 (US1)** depends on **Phase 1** and **Phase 2**.
-   **User Story 2 (US2)** depends on **User Story 1 (US1)** (as it reuses and extends the same agents).
-   **Data Ingestion (Phase 5)** can be developed in parallel with **Phase 3** and **Phase 4** but must be completed before they can be fully tested.

## Parallel Execution

-   Within **Phase 3 (US1)**, the agents can be developed in parallel to some extent, but the orchestrator depends on them.
-   **Phase 5 (Data Ingestion)** can be developed in parallel with the chat features.
-   All tasks in **Phase 6 (Polish)** are marked with `[P]` and can be worked on in parallel.

## Implementation Strategy

The implementation will follow a story-based, incremental approach.
1.  First, implement the foundational setup.
2.  Next, deliver User Story 1, providing the core book-wide chat functionality. This will be the Minimum Viable Product (MVP).
3.  Then, implement User Story 2 to add the selection-only feature.
4.  Finally, address polish and cross-cutting concerns.
