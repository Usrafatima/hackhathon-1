from typing import List
import re

class TextChunker:
    def __init__(self, chunk_size: int = 512, chunk_overlap: int = 100):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def chunk_text(self, text: str) -> List[str]:
        # Simple recursive character text splitter imitation
        # In a real-world scenario, a more sophisticated library (e.g., Langchain's RecursiveCharacterTextSplitter)
        # would be used, but for this task, a basic split and overlap is sufficient.

        chunks = []
        start_index = 0
        while start_index < len(text):
            end_index = start_index + self.chunk_size
            if end_index > len(text):
                chunks.append(text[start_index:])
                break

            chunk = text[start_index:end_index]
            chunks.append(chunk)

            start_index += (self.chunk_size - self.chunk_overlap)
            if start_index >= len(text):
                break
        return chunks
