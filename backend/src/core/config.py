from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    COHERE_API_KEY: str
    QDRANT_API_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str

    class Config:
        env_file = ".env"

settings = Settings()
