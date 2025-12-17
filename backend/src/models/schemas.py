from pydantic import BaseModel
from typing import List, Optional
import uuid

# ===============================================================================
# API Request Schemas
# ===============================================================================

class BookChatRequest(BaseModel):
    query: str
    session_id: Optional[uuid.UUID] = None

class SelectionChatRequest(BaseModel):
    query: str
    selected_text: str
    session_id: Optional[uuid.UUID] = None

class QuizGenerateRequest(BaseModel):
    topic: str
    num_questions: Optional[int] = 3

# ===============================================================================
# API Response Schemas
# ===============================================================================

class Source(BaseModel):
    chapter: str
    section: str
    url: str

class ChatResponse(BaseModel):
    response_text: str
    sources: List[Source]
    retrieved_context: Optional[List[str]] = None  # ← Change List[dict] to List[str]
    
class QuizQuestion(BaseModel):
    question: str
    options: List[str]
    correct_answer: str

class QuizResponse(BaseModel):
    questions: List[QuizQuestion]

class ErrorResponse(BaseModel):
    detail: str

# ===============================================================================
# Internal Data Models
# ===============================================================================

class ContentChunkMetadata(BaseModel):
    source_path: str
    chapter: str
    section: str
    chunk_index: int

class ContentChunk(BaseModel):
    raw_text: str
    metadata: ContentChunkMetadata