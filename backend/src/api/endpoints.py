from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks
from sqlalchemy.orm import Session
from typing import List

from src.models.schemas import (
    BookChatRequest,
    SelectionChatRequest,
    ChatResponse,
    Source,
    QuizGenerateRequest,
    QuizResponse
)
from src.services.rag_service import RAGOrchestrator
from src.db.database import get_db
from src.db.models import ChatSession, ChatMessage
from src.pipeline.process_markdown import load_markdown_documents, chunk_documents, Document as MarkdownDocument
from src.agents.quiz_agent import QuizAgent

router = APIRouter()

rag_orchestrator = RAGOrchestrator()
quiz_agent = QuizAgent()

DOCS_ROOT = "../frontend/docs"  # Relative path from backend/


# -------------------------
# Background Ingestion
# -------------------------
def _run_ingestion_process():
    print(f"Loading documents from: {DOCS_ROOT}")
    raw_docs = load_markdown_documents(DOCS_ROOT)
    print(f"Loaded {len(raw_docs)} documents.")

    if not raw_docs:
        print("No markdown documents found to ingest.")
        return

    print("Chunking documents...")
    chunked_docs = chunk_documents(raw_docs)
    print(f"Created {len(chunked_docs)} chunks.")

    from src.services.ingestion.load import QdrantLoader
    qdrant_loader = QdrantLoader()

    all_chunks_to_upload = []
    for doc in chunked_docs:
        if isinstance(doc, MarkdownDocument) or (hasattr(doc, "page_content") and hasattr(doc, "metadata")):
            all_chunks_to_upload.append({
                "raw_text": doc.page_content,
                "metadata": doc.metadata
            })
        else:
            print(f"Warning: Unexpected document type during ingestion: {type(doc)}")

    if all_chunks_to_upload:
        qdrant_loader.upload_chunks(all_chunks_to_upload)

    print("Ingestion process completed.")


@router.post("/ingest", status_code=status.HTTP_202_ACCEPTED)
def ingest_content(background_tasks: BackgroundTasks):
    background_tasks.add_task(_run_ingestion_process)
    return {"message": "Ingestion process started in the background."}


# -------------------------
# Book-Wide Chat
# -------------------------
@router.post("/chat/book", response_model=ChatResponse)
def chat_book(request: BookChatRequest, db: Session = Depends(get_db)):
    # 1. Get or create chat session
    session_id = request.session_id
    if session_id is None:
        chat_session = ChatSession()
        db.add(chat_session)
        db.commit()
        db.refresh(chat_session)
        session_id = chat_session.id
    else:
        chat_session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
        if not chat_session:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chat session not found.")

    try:
        # 2. Run RAG orchestrator
        orchestrator_response = rag_orchestrator.execute_book_wide_chat(request.query)
        retrieved_docs = orchestrator_response.get("retrieved_context", [])  # List of MarkdownDocument objects
        final_response_text = orchestrator_response.get("response_text", "")
        llm_prompt_used = orchestrator_response.get("llm_prompt", "")
        is_refusal = orchestrator_response.get("is_refusal", False)

        # 3. Prepare DB logging — store only text content
        retrieved_context_payload = []
        for doc in retrieved_docs:
            if hasattr(doc, "page_content"):
                retrieved_context_payload.append(doc.page_content)

        chat_message = ChatMessage(
            session_id=session_id,
            interaction_mode="book-wide",
            user_query=request.query,
            retrieved_context=retrieved_context_payload,
            llm_prompt=llm_prompt_used,
            final_response=final_response_text,
            is_refusal=is_refusal
        )
        db.add(chat_message)
        db.commit()
        db.refresh(chat_message)

        # 4. Prepare API response sources and context
        sources = []
        retrieved_context_for_api = []

        for doc in retrieved_docs:
            if hasattr(doc, "page_content") and hasattr(doc, "metadata"):
                text_content = doc.page_content
                metadata = doc.metadata

                retrieved_context_for_api.append(text_content)

                chapter = metadata.get("chapter", "Unknown Chapter")
                section = metadata.get("section", "Unknown Section")
                source_url = f"/docs/{chapter}/{section}".replace(" ", "-").lower()

                sources.append(Source(
                    chapter=chapter,      # ← Fixed
                    section=section,      # ← Fixed
                    url=source_url
                ))

        return ChatResponse(
            response_text=final_response_text,
            sources=sources,
            retrieved_context=retrieved_context_for_api
        )

    except Exception as e:
        print(f"Error in chat_book: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Sorry, I'm having trouble connecting right now. Please try again in a moment."
        )


# -------------------------
# Selection Chat
# -------------------------
@router.post("/chat/selection", response_model=ChatResponse)
def chat_selection(request: SelectionChatRequest, db: Session = Depends(get_db)):
    session_id = request.session_id
    if session_id is None:
        chat_session = ChatSession()
        db.add(chat_session)
        db.commit()
        db.refresh(chat_session)
        session_id = chat_session.id
    else:
        chat_session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
        if not chat_session:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Chat session not found.")

    try:
        orchestrator_response = rag_orchestrator.execute_selection_chat(request.query, request.selected_text)
        final_response_text = orchestrator_response.get("response_text", "")
        llm_prompt_used = orchestrator_response.get("llm_prompt", "")
        is_refusal = orchestrator_response.get("is_refusal", False)

        # Store selection context as text
        retrieved_context_payload = []
        retrieved_context = orchestrator_response.get("retrieved_context", [])
        for ctx in retrieved_context:
            if isinstance(ctx, dict) and "payload" in ctx:
                retrieved_context_payload.append(ctx["payload"].get("raw_text", ""))

        chat_message = ChatMessage(
            session_id=session_id,
            interaction_mode="selection-only",
            user_query=request.query,
            retrieved_context=retrieved_context_payload,
            llm_prompt=llm_prompt_used,
            final_response=final_response_text,
            is_refusal=is_refusal
        )
        db.add(chat_message)
        db.commit()
        db.refresh(chat_message)

        sources: List[Source] = []

        return ChatResponse(
            response_text=final_response_text,
            sources=sources,
            retrieved_context=retrieved_context_payload
        )

    except Exception as e:
        print(f"Error in chat_selection: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Sorry, I'm having trouble connecting right now. Please try again in a moment."
        )


# -------------------------
# Quiz Generation
# -------------------------
@router.post("/quiz/generate", response_model=QuizResponse)
def generate_quiz(request: QuizGenerateRequest):
    quiz_questions = quiz_agent.generate_quiz(request.topic, request.num_questions)
    if not quiz_questions:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Quiz generation failed.")
    return QuizResponse(questions=quiz_questions)