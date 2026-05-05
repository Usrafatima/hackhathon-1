from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os
from pathlib import Path

from src.api.endpoints import router as api_router
from src.db.database import engine, Base
from src.pipeline.process_markdown import load_markdown_documents, chunk_documents, Document as MarkdownDocument
from src.services.ingestion.load import QdrantLoader
from src.api.middleware import exception_handler

# -------------------------
# Load environment variables
# -------------------------
dotenv_path = Path(os.path.dirname(__file__)) / '.env'
load_dotenv(dotenv_path=dotenv_path)

# -------------------------
# Ingestion Logic
# -------------------------
def run_ingestion_on_startup():
    """
    Loads, chunks, and ingests markdown documents into the vector database.
    """
    DOCS_ROOT = "../frontend/docs"
    print(f"Loading documents from: {DOCS_ROOT}")
    raw_docs = load_markdown_documents(DOCS_ROOT)
    print(f"Loaded {len(raw_docs)} documents.")

    if not raw_docs:
        print("No markdown documents found to ingest.")
        return

    print("Chunking documents...")
    chunked_docs = chunk_documents(raw_docs)
    print(f"Created {len(chunked_docs)} chunks.")

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
        try:
            qdrant_loader.upload_chunks(all_chunks_to_upload)
        except Exception as e:
            print(f"FATAL: An unexpected error occurred during ingestion. The vector database may be incomplete. Error: {e}")

    print("Ingestion process completed.")


# -------------------------
# FastAPI lifespan
# -------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Application startup...")

    # Initialize relational database schema
    print("Initializing database schema...")
    Base.metadata.create_all(bind=engine)
    print("Database schema initialized.")

    # Run ingestion synchronously on startup
    print("Starting data ingestion...")
    run_ingestion_on_startup()
    print("Data ingestion complete.")

    yield

    print("Application shutdown...")


# -------------------------
# FastAPI app instance
# -------------------------
app = FastAPI(
    title="Humanoid Robot Book - RAG API",
    description="API for the RAG chatbot powered by Cohere and Qdrant.",
    version="1.0.0",
    lifespan=lifespan
)

# -------------------------
# Middleware
# -------------------------
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # React frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.middleware("http")(exception_handler)

# -------------------------
# Routers
# -------------------------
app.include_router(api_router, prefix="/api/v1")

# -------------------------
# Root endpoint
# -------------------------
@app.get("/")
async def root():
    return {"message": "Welcome to the RAG API. See /docs for details."}

# -------------------------
# Optional: test endpoint for Cohere
# -------------------------
@app.get("/test-cohere")
async def test_cohere():
    try:
        response = co.generate(
            model="command-light-1",
            prompt="Hello from test endpoint",
            max_tokens=10
        )
        return {"response": response.text}
    except Exception as e:
        return {"error": str(e)}
