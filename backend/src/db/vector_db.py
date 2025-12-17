from qdrant_client import QdrantClient
from src.core.config import settings

COLLECTION_NAME = "humanoid-robot-book"

qdrant_client = QdrantClient(
    url=settings.QDRANT_API_URL, 
    api_key=settings.QDRANT_API_KEY,
    timeout=60,
)

def get_qdrant_client():
    return qdrant_client