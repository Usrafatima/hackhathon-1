# Implementation Plan: RAG Chatbot

**Feature Branch**: `004-rag-chatbot`
**Feature Spec**: [spec.md](./spec.md)

This document outlines the technical implementation plan for the RAG Chatbot feature.

## Technical Context & Decisions

-   **Language**: Python 3.11+
-   **Framework**: FastAPI
-   **Vector Database**: Qdrant Cloud (Free Tier)
-   **Relational Database**: Neon Serverless Postgres
-   **LLM & Embeddings**: Cohere API
-   **Key Decisions**: Key technical decisions regarding chunking, retrieval, and architecture are documented in [research.md](./research.md).
-   **Data Models**: Detailed schemas for all data stores are documented in [data-model.md](./data-model.md).
-   **API Contracts**: The API is defined by an OpenAPI specification in [contracts/api.yaml](./contracts/api.yaml).

## Constitution Check

-   **I. Content Fidelity**: Adhered to. The entire design is centered around preventing hallucinations and ensuring all responses are grounded in the book's content. The Context Validation Agent is a key component for this.
-   **II. Modular and Clean Architecture**: Adhered to. The proposed three-agent design (Retrieval, Validation, Generation) provides clear separation of concerns and well-defined responsibilities.
-   **III. Structured Educational Design**: Not directly applicable to the chatbot feature itself, but the chatbot will consume and reference the book's structured content.
-   **IV. Testability and Reproducibility**: Adhered to. Each agent can be tested independently. The API-first design with a clear contract ensures testability. The detailed logging ensures reproducibility.
-   **V. Seamless AI Integration**: Adhered to. The plan details the integration of Cohere for embeddings and generation in a seamless RAG pipeline.

## Phase 1: Foundation & Architecture

-   **Deliverables**:
    -   Finalized `data-model.md`.
    -   Diagram of the agent-based architecture and data flow.
-   **Tasks**:
    1.  Set up Python project with FastAPI using Poetry.
    2.  Define Pydantic models for all API request/response schemas based on `contracts/api.yaml`.
    3.  Implement database connection modules for Qdrant and Neon Postgres.

## Phase 2: Ingestion & Data Preparation

-   **Deliverables**:
    -   A script or API endpoint (`/ingest`) to perform the full data ingestion pipeline.
    -   Populated Qdrant collection with book content.
-   **Tasks**:
    1.  Implement the text chunking logic based on the strategy in `research.md`.
    2.  Implement a service to generate embeddings for text chunks using the Cohere API.
    3.  Implement the logic to batch-upload vectors and their metadata payloads to the Qdrant collection.
    4.  Set up the `chat_sessions` and `chat_messages` tables in Neon Postgres using a migration tool like Alembic.

## Phase 3: Retrieval & Agent Implementation

-   **Deliverables**:
    -   Implemented `RetrievalAgent`, `ContextValidationAgent`, and `AnswerGenerationAgent`.
-   **Tasks**:
    1.  **Retrieval Agent**: Implement the logic to embed a query and retrieve `top-k=5` chunks from Qdrant.
    2.  **Context Validation Agent**:
        -   Implement the guardrail to check if retrieved context is sufficient.
        -   Implement the logic to handle both retrieved context (book-wide) and user-provided text (selection-only).
    3.  **Answer Generation Agent**:
        -   Implement prompt engineering to combine the user query and the context effectively.
        -   Implement the logic to call the Cohere Generate API.
        -   Implement the refusal logic when flagged by the validation agent.
    4.  Create an orchestrator service that wires the three agents together.

## Phase 4: API & Integration

-   **Deliverables**:
    -   Functional FastAPI endpoints: `/ingest`, `/chat/book`, `/chat/selection`.
-   **Tasks**:
    1.  Implement the `/ingest` endpoint to trigger the ingestion pipeline as a background task.
    2.  Implement the `/chat/book` endpoint, which uses the full agent orchestrator.
    3.  Implement the `/chat/selection` endpoint, ensuring it bypasses the Retrieval Agent.
    4.  Implement error handling to return generic 500 errors as per the spec.
    5.  The API response for chat endpoints will be immediate. The frontend will be responsible for displaying the "three dots" animation while waiting for the response.

## Phase 5: Observability & Logging

-   **Deliverables**:
    -   Structured logs for every chat request.
-   **Tasks**:
    1.  Integrate the `chat_messages` table from `data-model.md` into the chat endpoints.
    2.  Before returning a response, save all required data points (query, context, prompt, response, etc.) to the `chat_messages` table.
    3.  Ensure refusal events are logged correctly with the `is_refusal` flag.

## Phase 6: Testing & Validation

-   **Deliverables**:
    -   Unit and integration test suites.
    -   Acceptance test report.
-   **Tasks**:
    1.  **Unit Tests**: Write unit tests for each agent, the chunking logic, and individual services.
    2.  **Integration Tests**: Write tests to validate the full RAG pipeline from query to response for both book-wide and selection-only modes.
    3.  **Acceptance Tests**:
        -   Manually or with a script, test a set of pre-defined questions.
        -   Verify that responses are grounded in the provided context.
        -   Verify that citation hyperlinks are correctly formatted (the API will provide the URL path, the frontend will render the link).
        -   Verify that out-of-scope questions are refused.

## Phase 7: Deployment & Monitoring

-   **Deliverables**:
    -   Deployed API service.
    -   Basic monitoring dashboard.
-   **Tasks**:
    1.  Containerize the FastAPI application using Docker.
    2.  Deploy the container to a cloud service (e.g., Heroku, Fly.io, or a dedicated cloud provider).
    3.  Set up basic monitoring to track API latency, error rates, and costs for Cohere API usage.