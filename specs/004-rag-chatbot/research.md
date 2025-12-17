# Research & Decisions for RAG Chatbot Implementation

This document outlines key technical decisions made during the planning phase to resolve underspecified areas of the feature specification.

## 1. Text Chunking Strategy

-   **Decision**: Use a recursive character text splitting strategy with a chunk size of 512 tokens and an overlap of 100 tokens.
-   **Rationale**:
    -   Recursive splitting is robust. It attempts to split text along semantic boundaries (paragraphs, sentences, then words), which helps in preserving the contextual integrity of the chunks.
    -   A 512-token chunk size is a good balance for Cohere's embedding models and the context window sizes of modern generation models. It's large enough to contain meaningful context but small enough to allow multiple chunks in the final prompt.
    -   A 100-token overlap between chunks ensures that context is not lost at the boundaries of chunks, which is crucial for questions that span across chunk divisions.
-   **Alternatives Considered**:
    -   **Fixed-size chunking**: Simpler to implement but has a high risk of breaking sentences and losing semantic meaning.
    -   **Semantic chunking**: More advanced and potentially more effective, but requires more complex implementation and processing, which is not ideal for the initial implementation.

## 2. Top-k for Vector Retrieval

-   **Decision**: Set the default value of `k` (the number of retrieved chunks) to 5.
-   **Rationale**:
    -   Retrieving 5 chunks provides a rich source of context for the Language Model to generate a comprehensive answer.
    -   It's a standard practice that balances providing sufficient information against the risk of introducing noise or exceeding the context window of the generation model.
    -   The relevance scores provided by Qdrant for these 5 chunks can be used to further filter or prioritize the context if needed during the augmentation step.
-   **Alternatives Considered**:
    -   `k=3`: Might not provide enough context for more complex or nuanced questions.
    -   `k=10`: Increases the risk of including irrelevant information (noise) and makes the prompt sent to the LLM significantly larger, which can increase cost and latency.

## 3. Agent-based Design Responsibilities

-   **Decision**: The system will be implemented using a three-agent design with clear separation of concerns.
    1.  **Retrieval Agent**: Its sole responsibility is to interact with the vector database. It receives a user query, embeds it using the Cohere API, and queries Qdrant to fetch the `top-k` most relevant `ContentChunk` documents.
    2.  **Context Validation Agent**: This agent acts as a guardrail and formatter.
        -   It receives the retrieved chunks from the Retrieval Agent.
        -   It validates if the retrieved content is sufficient and relevant to answer the query. If not, it flags the query for a refusal.
        -   It formats the validated chunks into a clear string to be injected into the generator's prompt.
        -   In "selection-only" mode, this agent receives the user-selected text directly, bypassing the Retrieval Agent entirely.
    3.  **Answer Generation Agent**: This is the final agent in the chain.
        -   It receives the formatted context and the user query from the Context Validation Agent.
        -   It constructs the final, complete prompt for the Cohere Generate API.
        -   If the query was flagged for refusal, it generates the polite refusal message as specified in `FR-011`.
        -   It returns the final generated answer to the user.
-   **Rationale**: This modular, agent-based architecture aligns with the constitution's principle of a "Modular and Clean Architecture." It improves maintainability, allows for independent testing of each component, and makes the overall system easier to debug and extend.
-   **Alternatives Considered**:
    -   **Monolithic Function**: A single, large function could handle all the steps (retrieval, validation, generation). However, this would be difficult to test, maintain, and would violate the project's architectural principles.
