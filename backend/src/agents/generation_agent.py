import cohere
from typing import List, Dict, Any

from backend.src.core.config import settings
from backend.src.pipeline.process_markdown import Document

# --- CONSTANTS ---
NOT_FOUND_MESSAGE = "The answer is not present in the provided content."

# --- CLIENT ---
cohere_client = cohere.Client(settings.COHERE_API_KEY)

class GenerationAgent:
    """
    Generates a user-facing answer based on a query and context.
    """
    def __init__(self, cohere_client):
        self.cohere_client = cohere_client

    def _build_prompt(self, query: str, context_docs: List[Document]) -> str:
        """
        Builds the final prompt for the generation model, including context.
        """
        if not context_docs:
            return ""

        context_str = "\n\n".join(
            f"Source: {doc.metadata.get('source_path', 'N/A')}\nContent: {doc.page_content}"
            for doc in context_docs
        )

        prompt = f"""
        You are an expert assistant for the 'Humanoid Robot Book'. Your task is to answer questions based ONLY on the provided context below. Do not use any external knowledge.

        CONTEXT:
        ---
        {context_str}
        ---

        QUESTION: {query}

        INSTRUCTIONS:
        1. Analyze the provided context and the question carefully.
        2. Formulate a concise answer based strictly on the information in the context.
        3. If the answer is not found in the context, you MUST respond with the exact phrase: "{NOT_FOUND_MESSAGE}"
        4. Cite the sources from the context that support your answer.
        """
        return prompt

    def generate_answer(self, query: str, context_docs: List[Document]) -> Dict[str, Any]:
        """
        Reranks context, builds a prompt, and generates a final answer.

        Args:
            query: The user's question.
            context_docs: A list of Document objects from the retrieval step.

        Returns:
            A dictionary containing the answer and source documents.
        """
        if not context_docs:
            return {"answer": NOT_FOUND_MESSAGE, "sources": []}

        # 1. Rerank the documents to find the most relevant ones
        docs_to_rerank = [doc.page_content for doc in context_docs]
        
        print(f"Reranking {len(docs_to_rerank)} documents...")
        rerank_results = self.cohere_client.rerank(
            query=query,
            documents=docs_to_rerank,
            top_n=3, # Rerank and keep top 3
            model="rerank-english-v2.0"
        )

        # Filter documents with a relevance score above a threshold
        reranked_docs = []
        for hit in rerank_results:
            if hit.relevance_score > 0.3:
                # Find the original document to preserve metadata
                original_doc = context_docs[hit.index]
                reranked_docs.append(original_doc)
        
        if not reranked_docs:
            print("No relevant documents found after reranking.")
            return {"answer": NOT_FOUND_MESSAGE, "sources": []}

        print(f"Found {len(reranked_docs)} relevant documents after reranking.")

        # 2. Build the prompt for the generation model
        prompt = self._build_prompt(query, reranked_docs)
        if not prompt:
             return {"answer": NOT_FOUND_MESSAGE, "sources": []}

        # 3. Call the Cohere Chat API
        print("Generating answer with Cohere Chat API...")
        response = self.cohere_client.chat(
            message=prompt,
            model="command-r",
            temperature=0.1,
        )
        
        answer = response.text.strip()
        
        # Final check to ensure the model didn't ignore instructions
        if NOT_FOUND_MESSAGE in answer:
             final_answer = NOT_FOUND_MESSAGE
             sources = []
        else:
            final_answer = answer
            sources = [doc.metadata for doc in reranked_docs]


        return {"answer": final_answer, "sources": sources}

# --- AGENT INSTANCE ---
generation_agent = GenerationAgent(cohere_client)

if __name__ == '__main__':
    # Manual test for the generation agent
    test_query = "What is a digital twin?"
    
    # Mock context documents
    mock_docs = [
        Document(page_content="A digital twin is a virtual model of a physical object. It is used for simulations.", metadata={"source_path": "docs/Module2/1-intro.md"}),
        Document(page_content="ROS 2 is a set of software libraries and tools that help you build robot applications.", metadata={"source_path": "docs/Module1/1-intro.md"}),
        Document(page_content="Unity is a game engine often used for 3D rendering in robotics simulations.", metadata={"source_path": "docs/Module2/3-unity-rendering.md"}),
    ]

    result = generation_agent.generate_answer(test_query, mock_docs)
    
    print("\n--- Generation Result ---")
    print(f"Answer: {result['answer']}")
    print(f"Sources: {result['sources']}")
    print("------------------------")

    # Test "not found" case
    test_query_not_found = "What is the capital of France?"
    result_not_found = generation_agent.generate_answer(test_query_not_found, mock_docs)
    print("\n--- Not Found Test ---")
    print(f"Answer: {result_not_found['answer']}")
    print(f"Sources: {result_not_found['sources']}")
    print("------------------------")
