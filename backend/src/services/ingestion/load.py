from typing import List, Dict, Any
import uuid

from qdrant_client import QdrantClient, models
from src.db.vector_db import get_qdrant_client
from src.models.schemas import ContentChunkMetadata
from src.core.config import settings
import cohere

class QdrantLoader:
    def __init__(self, collection_name: str = "humanoid-robot-book"):
        self.qdrant_client: QdrantClient = get_qdrant_client()
        self.collection_name = collection_name
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Deletes and recreates the collection with 384 dim for Cohere light v3.0
        """
        try:
            self.qdrant_client.delete_collection(collection_name=self.collection_name)
            print(f"Deleted old collection '{self.collection_name}'")
        except:
            pass  # Collection didn't exist

        self.qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
        )
        print(f"Created new collection '{self.collection_name}' with size 384 for Cohere light model.")

    def upload_chunks(self, chunks: List[Dict[str, Any]]):
        """
        Embeds chunks using Cohere and uploads them to Qdrant.
        """
        if not chunks:
            return

        documents = [chunk['raw_text'] for chunk in chunks]

        # Use Cohere for embedding — same as query time
        response = self.cohere_client.embed(
            texts=documents,
            model="embed-english-light-v3.0",        # Same as query
            input_type="search_document"             # Best for stored docs
        )
        embeddings = response.embeddings

        points = []
        for i, chunk in enumerate(chunks):
            payload = ContentChunkMetadata(**chunk['metadata']).model_dump()
            payload["text_chunk"] = chunk['raw_text']  # Consistent with retrieval

            point = models.PointStruct(
                id=str(uuid.uuid4()),
                payload=payload,
                vector=embeddings[i]  # Already list of floats
            )
            points.append(point)

        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points,
            wait=True
        )
        print(f"Uploaded {len(points)} points to Qdrant.")