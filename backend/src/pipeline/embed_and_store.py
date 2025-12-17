import cohere
from qdrant_client import QdrantClient, models
from typing import List
import uuid

from backend.src.core.config import settings
from backend.src.db.vector_db import COLLECTION_NAME, get_qdrant_client
from backend.src.pipeline.process_markdown import Document, load_markdown_documents, chunk_documents

# --- COHERE CLIENT ---
cohere_client = cohere.Client(settings.COHERE_API_KEY)


def embed_and_store_chunks(
    chunks: List[Document],
    collection_name: str = COLLECTION_NAME
):
    """
    Generates embeddings for a list of document chunks and upserts them into Qdrant.
    """
    qdrant_client = get_qdrant_client()
    batch_size = 96  # Cohere's API limit for embeddings

    # TEMPORARY: Delete old collection to reset vector dimension (remove after first successful run)
    try:
        qdrant_client.delete_collection(collection_name=collection_name)
        print(f"Deleted old collection '{collection_name}' to avoid dimension conflict.")
    except Exception as e:
        print(f"Collection '{collection_name}' did not exist or could not be deleted: {e}")

    # Ensure collection exists with correct dimension (384 for light model)
    vector_size = 384  # embed-english-light-v3.0 outputs 384 dimensions
    try:
        qdrant_client.get_collection(collection_name=collection_name)
        print(f"Collection '{collection_name}' already exists.")
    except Exception:
        print(f"Creating new collection '{collection_name}' with vector size {vector_size}...")
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=models.Distance.COSINE
            )
        )

    # Now proceed with embedding and upserting
    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i:i + batch_size]
        texts_to_embed = [chunk.page_content for chunk in batch_chunks]

        print(f"Embedding batch {i // batch_size + 1} ({len(batch_chunks)} chunks)...")
        
        # Generate embeddings using Cohere — CORRECT settings for consistency
        response = cohere_client.embed(
            texts=texts_to_embed,
            model="embed-english-light-v3.0",        # ← 384 dimensions
            input_type="search_document"             # ← Best for stored document chunks
        )
        embeddings = response.embeddings

        # Verify dimension (debug print — you can remove later)
        if embeddings:
            print(f"First embedding dimension: {len(embeddings[0])}")

        # Prepare points for Qdrant
        points = []
        for chunk, embedding in zip(batch_chunks, embeddings):
            payload = chunk.metadata.copy()
            payload['text_chunk'] = chunk.page_content
            points.append(
                models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload=payload
                )
            )

        print(f"Upserting {len(points)} points to Qdrant...")
        
        # Upsert to Qdrant
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points,
            wait=True
        )

    print("Embedding and storage process completed.")


def run_ingestion_pipeline(source_dir: str):
    """
    Full pipeline: Loads, chunks, embeds, and stores documents from a source directory.
    """
    print(f"Starting ingestion pipeline for directory: {source_dir}")
    
    # 1. Load documents
    raw_docs = load_markdown_documents(source_dir)
    print(f"Loaded {len(raw_docs)} documents.")
    
    # 2. Chunk documents
    chunks = chunk_documents(raw_docs)
    print(f"Created {len(chunks)} chunks.")
    
    # 3. Embed and store
    if chunks:
        embed_and_store_chunks(chunks)
    else:
        print("No chunks were created. Nothing to embed.")


if __name__ == '__main__':
    # Example usage
    docs_path = 'frontend/static/docs'
    run_ingestion_pipeline(docs_path)