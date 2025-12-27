from qdrant_client import QdrantClient
from typing import List
from fastembed import TextEmbedding

from src.db.vector_db import COLLECTION_NAME, get_qdrant_client
from src.pipeline.process_markdown import Document

# --- CLIENTS ---
# The qdrant_client is initialized here. The embedding model is handled within the agent.
qdrant_client = get_qdrant_client()

# Embedding Model Configuration
EMBEDDING_DIMENSION = 384

class QdrantRetrievalAgent:
    """
    Handles the retrieval of relevant documents from the vector store using FastEmbed.
    """

    def __init__(self, qdrant_client, collection_name: str = COLLECTION_NAME):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name
        self.embedding_model = TextEmbedding()

    def execute(self, query: str, top_k: int = 50) -> List[Document]:
        """
        Embeds a query using FastEmbed and performs a semantic search in Qdrant.
        """
        print(f"Retrieving documents for query: '{query}'")

        # 1. Embed the query using FastEmbed
        # Convert generator to list to access the first element
        query_embedding = list(self.embedding_model.embed([query]))[0]

        # 2. Use the search method for semantic search
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding.tolist(),  # Convert numpy array to list
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # 3. Format results into Document objects
        retrieved_docs = []
        for result in search_results:
            payload = result.payload or {}
            text_chunk = payload.get("text_chunk", "")
            metadata = payload.copy()
            metadata.pop("text_chunk", None)

            retrieved_docs.append(
                Document(page_content=text_chunk, metadata=metadata)
            )

        print(f"Retrieved {len(retrieved_docs)} documents.")
        return retrieved_docs