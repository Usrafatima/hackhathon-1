from typing import List, Dict, Any, Tuple

# If you have a base class, keep this import — otherwise remove it
# from src.services.agents.base import ContextValidationAgent

# If you don't actually have a base class file yet, just inherit from object
class SimpleContextValidationAgent:  # ← Removed inheritance if base doesn't exist
    def execute(self, query: str, context: List[Any], mode: str = "book-wide") -> Tuple[str, bool]:
        """
        Validates the retrieved context and formats it for the generation agent.
        Returns a tuple of (formatted_context_string, is_sufficient_bool).
        """
        if mode == "selection-only":
            raw_text = context[0]['payload']['raw_text']
            is_sufficient = bool(raw_text and len(raw_text.strip()) > 50)
            return raw_text, is_sufficient

        # Book-wide mode: handle LangChain Document objects from Qdrant retrieval
        formatted_context = ""
        actual_content_length = 0

        for i, doc in enumerate(context):
            # Support both Document objects and dict format
            if hasattr(doc, "page_content") and hasattr(doc, "metadata"):
                raw_text = doc.page_content
                score = doc.metadata.get("score", 0.0)  # Qdrant adds score to metadata
            elif isinstance(doc, dict) and 'payload' in doc:
                raw_text = doc['payload']['raw_text']
                score = doc.get('score', 0.0)
            else:
                continue  # skip unexpected format

            formatted_context += f"--- Document {i+1} (Score: {score:.4f}) ---\n"
            formatted_context += raw_text.strip()
            formatted_context += "\n----------------------------------------\n\n"
            actual_content_length += len(raw_text.strip())

        is_sufficient = actual_content_length > 100

        return formatted_context, is_sufficient