from typing import List
import cohere
from src.core.config import settings

class CohereEmbeddingService:
    def __init__(self):
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        response = self.cohere_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="clustering" # or "search_document" depending on usage
        )
        return response.embeddings