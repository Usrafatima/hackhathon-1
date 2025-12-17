import cohere
from qdrant_client import QdrantClient
from typing import List

from src.core.config import settings
from src.db.vector_db import COLLECTION_NAME, get_qdrant_client
from src.pipeline.process_markdown import Document  # Re-using the Document class

# --- CLIENTS ---
cohere_client = cohere.Client(settings.COHERE_API_KEY)
qdrant_client = get_qdrant_client()


class QdrantRetrievalAgent:
    """
    Handles the retrieval of relevant documents from the vector store using the modern Qdrant Query API.
    """

    def __init__(self, cohere_client, qdrant_client, collection_name: str = COLLECTION_NAME):
        self.cohere_client = cohere_client
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name

    def execute(self, query: str, top_k: int = 50) -> List[Document]:  # ← Increased from 5 to 20
        """
        Embeds a query and performs a semantic search in Qdrant using query_points.
        """
        print(f"Retrieving documents for query: '{query}'")

        # 1. Embed the query using Cohere — consistent with ingestion (384 dims)
        response = self.cohere_client.embed(
            texts=[query],
            model="embed-english-light-v3.0",        # ← Matches ingestion: 384 dimensions
            input_type="search_query"                # ← Correct type for queries
        )
        query_embedding = response.embeddings[0]

        # Debug: Print actual embedding dimension
        print(f"Query embedding dimension: {len(query_embedding)}")

        # 2. Use the modern query_points method (correct for qdrant-client 1.16+)
        search_results = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,       # ← 'query' parameter for dense vectors
            limit=top_k,                 # ← Now returns up to 20 results
            with_payload=True,
            with_vectors=False           # ← Saves bandwidth
        ).points

        # 3. Format results into Document objects
        retrieved_docs = []
        for result in search_results:
            payload = result.payload or {}
            text_chunk = payload.get("text_chunk", "")
            metadata = payload.copy()
            metadata.pop("text_chunk", None)  # Avoid duplication

            retrieved_docs.append(
                Document(page_content=text_chunk, metadata=metadata)
            )

        print(f"Retrieved {len(retrieved_docs)} documents.")
        return retrieved_docs