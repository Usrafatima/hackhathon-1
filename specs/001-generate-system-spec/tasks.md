# Tasks: Generate System Specification

**Input**: Design documents from `specs/001-generate-system-spec/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup and Foundation

- **Title**: Epic 1: Docusaurus + 16 Chapters
- **Description**: Initialize the frontend project with Docusaurus and create the basic structure for all 16 chapters.
- **Time Estimate**: 8 hours
- **Dependencies**: None
- **Priority**: High

### Sub-tasks
- [X] T001 Initialize Docusaurus project in `frontend/`
- [X] T002 Configure Docusaurus for 16 chapter routes in `frontend/docusaurus.config.js`
- [X] T003 [P] Create placeholder Markdown files for all 16 chapters in `frontend/docs/`
- [X] T004 [P] Populate the 16 chapter files with initial content from source material.

---

## Phase 2: Data and Backend Foundation

- **Title**: Epic 2, 3 & 4: Embedding Pipeline, Quadrant Setup, and FastAPI RAG Backend
- **Description**: Build the core backend systems for data ingestion, embedding, and retrieval.
- **Time Estimate**: 24 hours
- **Dependencies**: Phase 1
- **Priority**: High

### Sub-tasks
- [X] T005 Initialize FastAPI project in `backend/`
- [X] T006 [P] Implement a script to read and chunk Markdown files from `frontend/docs/` in `backend/src/pipeline/`
- [X] T007 [P] Set up a vector database and configure quadrant layout in `backend/src/db/`
- [X] T008 Implement embedding generation for content chunks in `backend/src/pipeline/`
- [X] T009 Create a RAG pipeline service in `backend/src/services/rag_service.py`
- [X] T010 Implement a `/query` endpoint in `backend/src/api/endpoints.py` that uses the RAG service.

---

## Phase 3: Core AI and User-Facing Features

- **Title**: Epic 5 & 6: Urdu Translation and Personalization
- **Description**: Implement the Urdu translation layer and user personalization features.
- **Time Estimate**: 16 hours
- **Dependencies**: Phase 2
- **Priority**: Medium

### Sub-tasks
- [X] T011 [P] Implement a user profile model and database schema in `backend/src/models/`
- [X] T012 [P] Create endpoints for user profile management in `backend/src/api/endpoints.py`
- [X] T013 Integrate user profile data into the RAG pipeline in `backend/src/services/rag_service.py`
- [X] T014 [P] Implement a translation service for UI and AI responses in `backend/src/services/translation_service.py`
- [ ] T015 [P] Create a middleware or decorator for handling the Urdu language toggle in `backend/src/api/`
- [ ] T016 [P] [US2] Add a language toggle button to the React chat widget.
- [X] T017 [US1] Implement the React Chat Widget in `frontend/src/components/ChatWidget.js`
- [X] T018 [US1] Integrate the Chat Widget with the Docusaurus layout in `frontend/src/theme/Root.js`
- [X] T019 [US1] Implement text selection detection and a query button in `frontend/src/components/`

---

## Phase 4: Sub-Agents

- **Title**: Epic 7, 8 & 9: Quiz, Lab, and Code Explainer Sub-Agents
- **Description**: Build the specialized sub-agents for generating quizzes, labs, and code explanations.
- **Time Estimate**: 20 hours
- **Dependencies**: Phase 2
- **Priority**: Medium

### Sub-tasks
- [X] T020 [P] Implement the Quiz Sub-Agent logic in `backend/src/agents/quiz_agent.py`
- [X] T021 [P] Create a `/quiz` endpoint for the Quiz Sub-Agent in `backend/src/api/endpoints.py`
- [X] T022 [P] Implement the Lab Sub-Agent logic in `backend/src/agents/lab_agent.py`
- [X] T023 [P] Create a `/lab` endpoint for the Lab Sub-Agent in `backend/src/api/endpoints.py`
- [X] T024 [P] Implement the Code Explainer Agent logic in `backend/src/agents/explain_agent.py`
- [X] T025 [P] Create an `/explain` endpoint for the Code Explainer Agent in `backend/src/api/endpoints.py`

---

## Phase 5: Deployment and Testing

- **Title**: Epic 10 & 11: Deployment Pipeline and Testing
- **Description**: Deploy the application and implement a comprehensive testing suite.
- **Time Estimate**: 24 hours
- **Dependencies**: All previous phases
- **Priority**: High

### Sub-tasks
- [X] T026 [P] Create a `Dockerfile` for the FastAPI backend.
- [X] T027 [P] Configure the frontend for deployment on Vercel.
- [X] T028 [P] Configure the backend for deployment on Railway, including WordCell storage.
- [X] T029 [P] Write unit tests for all backend services in `backend/tests/unit/`
- [X] T030 [P] Write integration tests for the API endpoints in `backend/tests/integration/`
- [X] T031 [P] Write unit tests for React components in `frontend/src/components/`
- [X] T032 [P] Write end-to-end tests for the main user flows.

## Dependencies & Execution Order
- **Phase 1** must be completed before any other phase.
- **Phase 2** can begin after Phase 1 is complete.
- **Phase 3 and 4** can be worked on in parallel after Phase 2 is complete.
- **Phase 5** can only begin after all other phases are complete.

## Implementation Strategy
The project will be developed in phases, with the Docusaurus site and chapter content being the first priority. The backend services and AI features will be built on top of this foundation. Sub-agents and user-facing features will be developed in parallel where possible. Finally, the application will be deployed and tested.
