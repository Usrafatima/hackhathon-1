from sqlalchemy import Column, Integer, String, Text, Boolean, DateTime, func, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID
import uuid
from src.db.database import Base

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    user_id = Column(String(255), nullable=True)

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    interaction_mode = Column(String(20), nullable=False)
    user_query = Column(Text, nullable=False)
    retrieved_context = Column(JSON, nullable=True)
    llm_prompt = Column(Text, nullable=False)
    final_response = Column(Text, nullable=False)
    is_refusal = Column(Boolean, nullable=False, default=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
