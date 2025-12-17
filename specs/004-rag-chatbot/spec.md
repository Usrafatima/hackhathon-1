# Feature Specification: AI-Native RAG Chatbot

**Feature Branch**: `004-rag-chatbot`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Target Audience Readers of a published technical book seeking accurate, context-aware responses strictly based on the book's content. Core Focus Implement an AI-native Retrieval-Augmented Generation (RAG) system. Embed the chatbot within the book's website. Enforce strict context adherence to eliminate hallucinations. Support two interaction modes: Full-book mode: Use the entire book as the knowledge base. Selection-only mode: Limit responses to user-selected text excerpts..."

## Clarifications

### Session 2025-12-15

- Q: Requirement FR-009 is to "log all major RAG steps". What specific information must be logged to meet the auditability and traceability requirement? → A: Log the user query, retrieved text chunks (including their scores), the exact prompt sent to the LLM, and the final generated response.
- Q: How should the system handle failures of external APIs (e.g., Cohere, Qdrant)? → A: Display a generic, friendly error message like, "Sorry, I'm having trouble connecting right now. Please try again in a moment."
- Q: What feedback should the user receive while the chatbot is processing a request? → A: Display a dynamic loading indicator (three dots '...') in the chat interface while the system is processing a request.
- Q: What are the definitive metadata fields for a 'Content Chunk' and their expected data types? → A: Attributes for Content Chunk metadata should include `chapter_number` (int), `chapter_title` (string), `section_number` (int), `section_title` (string), and `page_number` (int, optional).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Querying the Entire Book (Priority: P1)

A reader is studying Chapter 2 and is confused about a concept. They open the chat widget on the book's website and ask, "Can you explain the middleware for robot control?". The chatbot provides a concise explanation sourced directly from the book and includes a reference to the specific chapter and section where the information can be found.

**Why this priority**: This is the core functionality of the RAG system and provides the primary value to the user—getting immediate, contextually accurate answers to their questions about the book.

**Independent Test**: Can be tested by asking a question that is definitively answered in the book and verifying that the response is accurate, context-based, and includes a citation.

**Acceptance Scenarios**:

1.  **Given** the user is on the book's website,
    **When** they ask a question covered in the book's content,
    **Then** the system provides a contextually relevant answer derived solely from the book.
2.  **Given** the system has provided an answer,
    **When** the user reviews the answer,
    **Then** it includes a reference to the source chapter/section.
3.  **Given** the user is on the book's website,
    **When** they ask a question *not* covered in the book's content,
    **Then** the system politely refuses to answer, stating that the information is not available in the book.

---

### User Story 2 - Querying a Specific Text Selection (Priority: P2)

A reader highlights a specific, complex paragraph on the website and invokes the chatbot. They ask, "What does this paragraph mean for sensor simulation?". The chatbot analyzes *only the selected text* and provides an explanation based on that limited context.

**Why this priority**: This provides a focused, precision tool for users to drill down into specific parts of the text, which is a key requirement.

**Independent Test**: Can be tested by selecting a snippet of text, asking a question about it, and verifying the answer *only* uses information from that snippet.

**Acceptance Scenarios**:

1.  **Given** the user has selected a portion of text on the website,
    **When** they ask a question about that selection,
    **Then** the system provides an answer derived exclusively from the selected text.
2.  **Given** a user has asked a question about a selected text,
    **When** the question cannot be answered from the selection alone,
    **Then** the system politely refuses, stating it cannot answer based on the provided context.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST provide a chat interface embedded within the book's website.
- **FR-002**: The system MUST ingest and process the book's content into a searchable knowledge base.
- **FR-003**: The system MUST support a "full-book" mode where the entire book is used as context for answering queries.
- **FR-004**: The system MUST support a "selection-only" mode where only user-selected text is used as context.
- **FR-005**: In "full-book" mode, the system MUST retrieve text chunks relevant to the user's query from the knowledge base.
- **FR-006**: In "selection-only" mode, the system MUST NOT perform any vector retrieval from the knowledge base.
- **FR-007**: The system MUST generate all responses based *exclusively* on the context provided (either from retrieval or user selection).
- **FR-008**: The system MUST refuse to answer any query for which it has insufficient context.
- **FR-009**: For auditing and traceability, the system MUST log the following for each query: the user's original query, the retrieved text chunks (including their relevance scores), the exact and complete prompt sent to the LLM for generation, and the final generated response.
- **FR-010**: The system MUST include references to the source chapter or section in its responses as hyperlinks, navigating the user to the corresponding section on the webpage.
- **FR-011**: The system MUST display a generic, user-friendly error message (e.g., "Sorry, I'm having trouble connecting right now. Please try again in a moment.") to the user if an external API call (e.g., Cohere, Qdrant) fails.
- **FR-012**: The system MUST display a dynamic loading indicator (three dots '...') in the chat interface while a user's request is being processed.

### Key Entities

-   **Content Chunk**: A segment of text from the book. Attributes include the raw text and detailed metadata: `chapter_number` (integer), `chapter_title` (string), `section_number` (integer), `section_title` (string), and `page_number` (integer, optional).
-   **User Query**: The question submitted by the user. Attributes include the query text and the interaction mode (full-book or selection-only).
-   **Chat Response**: The answer generated by the system. Attributes include the response text and a list of source Content Chunks used.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 100% of generated answers must be factually grounded in the provided book content, verified through logging and review.
-   **SC-002**: In selection-only mode, 100% of responses must be derived *only* from the user-provided text excerpt.
-   **SC-003**: The system correctly refuses to answer at least 99% of out-of-scope queries during testing.
-   **SC-004**: End-to-end response time for 95% of queries (from submission to rendered response) is under 5 seconds.
-   **SC-005**: All retrieval, augmentation, and generation steps for every query are successfully logged and traceable.
