from typing import List, Dict, Any
import uuid

from qdrant_client import QdrantClient, models
from fastembed import TextEmbedding

from src.db.vector_db import get_qdrant_client
from src.models.schemas import ContentChunkMetadata

# Embedding Model Configuration
# Using Qdrant's fast-bge-small-en model by default, which is 384 dimensions
EMBEDDING_DIMENSION = 384

class QdrantLoader:
    def __init__(self, collection_name: str = "humanoid-robot-book"):
        self.qdrant_client: QdrantClient = get_qdrant_client()
        self.collection_name = collection_name
        self.embedding_model = TextEmbedding()
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Deletes and recreates the collection with the correct dimension for the FastEmbed model.
        """
        try:
            self.qdrant_client.delete_collection(collection_name=self.collection_name)
            print(f"Deleted old collection '{self.collection_name}'")
        except Exception:
            pass  # Collection didn't exist

        self.qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(size=EMBEDDING_DIMENSION, distance=models.Distance.COSINE)
        )
        print(f"Created new collection '{self.collection_name}' with size {EMBEDDING_DIMENSION} for FastEmbed model.")

    def upload_chunks(self, chunks: List[Dict[str, Any]]):
        """
        Embeds chunks using FastEmbed and uploads them to Qdrant.
        """
        if not chunks:
            return

        documents = [chunk['raw_text'] for chunk in chunks]

        # Use FastEmbed for embedding
        # Convert the generator to a list to allow for indexing
        embeddings = list(self.embedding_model.embed(documents))

        points = []
        for i, chunk in enumerate(chunks):
            payload = ContentChunkMetadata(**chunk['metadata']).model_dump()
            payload["text_chunk"] = chunk['raw_text']

            point = models.PointStruct(
                id=str(uuid.uuid4()),
                payload=payload,
                vector=embeddings[i].tolist() # Convert numpy array to list
            )
            points.append(point)

        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points,
            wait=True
        )
        print(f"Uploaded {len(points)} points to Qdrant.")