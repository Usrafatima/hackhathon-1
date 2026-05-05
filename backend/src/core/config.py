from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    HUGGINGFACE_API_TOKEN: str
    QDRANT_API_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str
    QDRANT_COLLECTION_NAME: str

    class Config:
        env_file = ".env"

settings = Settings()
