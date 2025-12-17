---
id: 001
title: Create RAG Chatbot Specification
stage: spec
date: 2025-12-15
surface: agent
model: gemini-1.5-pro-a2-0802
feature: 004-rag-chatbot
branch: 004-rag-chatbot
user: user
command: $ARGUMENTS
labels: ["specification", "rag", "chatbot"]
links:
  spec: specs/004-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-chatbot/spec.md
  - specs/004-rag-chatbot/checklists/requirements.md
tests:
  - No automated tests were run for specification generation.
---

## Prompt

Target Audience Readers of a published technical book seeking accurate, context-aware responses strictly based on the book's content. Core Focus Implement an AI-native Retrieval-Augmented Generation (RAG) system. Embed the chatbot within the book's website. Enforce strict context adherence to eliminate hallucinations. Support two interaction modes: Full-book mode: Use the entire book as the knowledge base. Selection-only mode: Limit responses to user-selected text excerpts. Success Criteria All chatbot responses must derive exclusively from the book's content. In selection-only mode, bypass vector retrieval entirely and rely solely on provided text. Explicitly refuse to answer if the query lacks sufficient context. Include references to chapter/section metadata in responses where applicable. Ensure all RAG processes (retrieval, augmentation, generation) are fully traceable and auditable. Constraints Backend: Use FastAPI (Python) exclusively. LLM: Integrate only with Cohere API. Vector Database: Qdrant Cloud (Free Tier). Relational Database: Neon Serverless Postgres. Development Tools: SpecKit + Gemini CLI for workflow. Security: Avoid hardcoded secrets in prompts or code; use environment variables. Real Environment Configuration (for Deployment) COHERE_API_KEY="Lqvb3CF9qAcXWiWHxvE2aFsrZGVjKJo68fvJ0TCT" NEON_DATABASE_URL="postgresql://neondb_owner:npg_tkMgvAo6xJ9S@ep-wild-cell-ad4pubdh-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require" QDRANT_API_URL="https://d6755c55-336c-47b3-857a-466174a9894e.europe-west3-0.gcp.cloud.qdrant.io" QDRANT_CLUSTER_ID="d6755c55-336c-47b3-857a-466174a9894e" RAG Behavior Requirements Book-Wide Mode: Embed queries using Cohere embeddings. Query Qdrant for top-k relevant chunks. Inject retrieved chunks as context into the Cohere generation prompt. Selection-Only Mode: Disable all vector search mechanisms. Use only the user-provided selected text as context. Politely refuse if the question cannot be answered from the given text alone. Verification Requirements Ground every response in the provided context, with no external knowledge permitted. Log all retrieval, validation, and generation steps for review. Make errors and refusals clear, user-friendly, and explicit (e.g., "I'm sorry, but I can't answer that based on the available context from the book."). Exclusions (Not Building) Open-domain or internet-connected features. Model fine-tuning or training. Book content generation. User recommendation or personalization. Analytics, tracking, or monetization. Project Timeline Phase 1: Develop detailed technical specification. Phase 2: Create implementation plan. Phase 3: Write production-grade code. Review and approve each phase before advancing.

## Response snapshot

- Created branch `004-rag-chatbot`.
- Generated `specs/004-rag-chatbot/spec.md` based on the provided feature description.
- Generated `specs/004-rag-chatbot/checklists/requirements.md` checklist.
- Clarified citation format with the user (chosen: Hyperlink).
- Updated `specs/004-rag-chatbot/spec.md` and `specs/004-rag-chatbot/checklists/requirements.md` accordingly.

## Outcome

- ✅ Impact: Specification for RAG chatbot feature created and clarified.
- 🧪 Tests: No automated tests were run for specification generation.
- 📁 Files: Created `spec.md` and `requirements.md`.
- 🔁 Next prompts: Next step is `/sp.plan` to create the implementation plan.
- 🧠 Reflection: Encountered persistent PowerShell argument parsing issues with `create-new-feature.ps1`, necessitating manual creation of branch and files. This highlights a fragility in the script's invocation method, especially with complex string inputs.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell argument parsing for `create-new-feature.ps1` with complex feature description strings.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Investigate robust ways to pass multi-line, complex strings as arguments to PowerShell scripts from the agent.
