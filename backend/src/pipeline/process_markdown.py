import os
from typing import List, Dict, Any
from langchain_text_splitters import RecursiveCharacterTextSplitter

class Document:
    """
    A simple container for a chunk of text and its associated metadata.
    """
    def __init__(self, page_content: str, metadata: Dict[str, Any]):
        self.page_content = page_content
        self.metadata = metadata

    def __repr__(self):
        return f"Document(page_content='{self.page_content[:50]}...', metadata={self.metadata})"


def load_markdown_documents(root_dir: str) -> List[Document]:
    """
    Recursively loads all markdown files from a directory into a list of Documents.
    Each Document represents a whole file for now.
    """
    documents = []
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(".md"):
                file_path = os.path.join(dirpath, filename)
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        text = f.read()
                    
                    # Basic metadata extraction from path
                    relative_path = os.path.relpath(file_path, root_dir)
                    path_parts = relative_path.split(os.sep)
                    
                    metadata = {
                        "source_path": relative_path,
                        "chapter": path_parts[0] if len(path_parts) > 0 else "Unknown",
                        "section": os.path.splitext(path_parts[-1])[0] if len(path_parts) > 0 else "Unknown"
                    }
                    
                    documents.append(Document(page_content=text, metadata=metadata))
                except Exception as e:
                    print(f"Error reading or processing {file_path}: {e}")
    return documents

def chunk_documents(documents: List[Document]) -> List[Document]:
    """
    Splits a list of Documents into smaller chunks using a text splitter.
    """
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=500,
        chunk_overlap=50,
        separators=["\n\n", "\n", " ", ""], # Markdown-friendly separators
    )
    
    chunked_docs = []
    for doc in documents:
        chunks = text_splitter.split_text(doc.page_content)
        for i, chunk in enumerate(chunks):
            # Each chunk inherits the metadata from its parent document
            chunked_doc = Document(
                page_content=chunk,
                metadata={
                    **doc.metadata,
                    "chunk_index": i
                }
            )
            chunked_docs.append(chunked_doc)
            
    return chunked_docs

if __name__ == '__main__':
    # Example usage for testing
    # Note: Adjust the path to be relative to the project root where you run the script
    # e.g., python -m backend.src.pipeline.process_markdown
    docs_path = 'frontend/static/docs'
    
    print(f"Loading documents from: {docs_path}")
    raw_docs = load_markdown_documents(docs_path)
    print(f"Loaded {len(raw_docs)} documents.")
    
    if raw_docs:
        print("\nChunking documents...")
        chunks = chunk_documents(raw_docs)
        print(f"Created {len(chunks)} chunks.")
        
        # Print a sample chunk
        if chunks:
            print("\n--- Sample Chunk ---")
            print(chunks[0].page_content)
            print(f"Metadata: {chunks[0].metadata}")
            print("--------------------")
