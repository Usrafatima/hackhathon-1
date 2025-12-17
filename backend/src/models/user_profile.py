from pydantic import BaseModel, Field
from typing import List, Optional

class UserProfile(BaseModel):
    id: Optional[str] = Field(None, description="Unique identifier for the user profile.")
    username: str = Field(..., description="The user's chosen username.")
    preferred_language: str = Field("en", description="The user's preferred language (e.g., 'en', 'ur').")
    reading_level: str = Field("intermediate", description="The user's reading level (e.g., 'beginner', 'intermediate', 'advanced').")
    interests: List[str] = Field(default_factory=list, description="A list of the user's interests in robotics.")
    chat_history_ids: List[str] = Field(default_factory=list, description="List of IDs referencing chat history entries.")

    class Config:
        schema_extra = {
            "example": {
                "username": "robot_enthusiast",
                "preferred_language": "en",
                "reading_level": "advanced",
                "interests": ["humanoid locomotion", "ros2", "computer vision"],
                "chat_history_ids": ["chat_123", "chat_456"]
            }
        }
