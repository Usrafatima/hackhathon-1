from typing import Dict, Any, List
from src.services.agents.retrieval_agent import QdrantRetrievalAgent, cohere_client, qdrant_client
from src.services.agents.validation_agent import SimpleContextValidationAgent
from src.services.agents.generation_agent import CohereAnswerGenerationAgent
from src.models.schemas import Source

class RAGOrchestrator:
    def __init__(self):
        self.retrieval_agent = QdrantRetrievalAgent(cohere_client, qdrant_client)
        self.validation_agent = SimpleContextValidationAgent()
        self.generation_agent = CohereAnswerGenerationAgent()

    def execute_book_wide_chat(self, query: str) -> Dict[str, Any]:
        """
        Orchestrates the RAG pipeline for the book-wide chat mode.
        """
        # 1. Retrieve context (list of Document objects)
        retrieved_docs = self.retrieval_agent.execute(query)

        # 2. Validate and format context
        formatted_context, is_sufficient = self.validation_agent.execute(query, retrieved_docs, mode="book-wide")

        is_refusal = False
        final_response_text = ""
        llm_prompt_used = ""

        if not is_sufficient:
            final_response_text = "I'm sorry, but I can't answer that based on the available context from the book."
            is_refusal = True
        else:
            # 3. Generate answer
            final_response_text, llm_prompt_used = self.generation_agent.execute(query, formatted_context)
            if "I'm sorry, but I can't answer that based on the available context from the book." in final_response_text:
                is_refusal = True

        # 4. Build sources safely
        sources: List[Source] = []
        for doc in retrieved_docs:
            if hasattr(doc, "metadata"):
                metadata = doc.metadata
                chapter = metadata.get("chapter", "Unknown Chapter")
                section = metadata.get("section", "Unknown Section")
                # Build source URL safely
                source_url = f"/docs/{chapter}/{section}".replace(" ", "-").lower()
                sources.append(Source(
                chapter=chapter,
                section=section,
                url=source_url
))

        return {
            "response_text": final_response_text,
            "sources": sources,
            "retrieved_context": retrieved_docs,  # still Document objects
            "llm_prompt": llm_prompt_used,
            "is_refusal": is_refusal
        }

    def execute_selection_chat(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Orchestrates the RAG pipeline for the selection-only chat mode.
        """
        context_doc = [{"payload": {"raw_text": selected_text}}]
        formatted_context, is_sufficient = self.validation_agent.execute(query, context_doc, mode="selection-only")

        is_refusal = False
        final_response_text = ""
        llm_prompt_used = ""

        if not is_sufficient:
            final_response_text = "I'm sorry, but I can't answer that based on the available context from the book."
            is_refusal = True
        else:
            final_response_text, llm_prompt_used = self.generation_agent.execute(query, formatted_context)
            if "I'm sorry, but I can't answer that based on the available context from the book." in final_response_text:
                is_refusal = True

        return {
            "response_text": final_response_text,
            "sources": [],  # No sources from vector DB
            "retrieved_context": context_doc,
            "llm_prompt": llm_prompt_used,
            "is_refusal": is_refusal
        }
