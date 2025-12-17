from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os
import cohere

from src.api.endpoints import router as api_router
from src.db.database import engine, Base
from src.services.ingestion.load import QdrantLoader
from src.api.middleware import exception_handler

# -------------------------
# Load environment variables
# -------------------------
load_dotenv()  # Load .env file

COHERE_API_KEY = os.getenv("5lkNIoDgAJ9BYWL23IwpFrgWeg3yJ6QLqZuNE4my")
if not COHERE_API_KEY:
    raise ValueError("Cohere API key not found in environment variables!")

# Initialize Cohere client
co = cohere.Client(COHERE_API_KEY)
print("Cohere client initialized successfully.")

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

    # Initialize vector database collection
    print("Initializing Qdrant collection...")
    _ = QdrantLoader()
    print("Qdrant collection initialized (or checked).")

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
