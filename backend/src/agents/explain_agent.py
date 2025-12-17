import os
from typing import Dict, Optional
from langchain_community.llms import Ollama
from langchain_core.prompts import PromptTemplate

from backend.src.db.vector_db import get_chroma_client, get_or_create_collection
from backend.src.services.rag_service import RAGService # Re-using RAGService for context retrieval
import json

class ExplainAgent:
    def __init__(self, model_name="llama2", embedding_model_name="all-MiniLM-L6-v2", chroma_path=None):
        self.rag_service = RAGService(model_name=model_name, embedding_model_name=embedding_model_name, chroma_path=chroma_path)
        self.llm = self.rag_service.llm # Use the same LLM instance from RAGService

        self.explain_prompt_template = PromptTemplate(
            template="""You are an AI assistant designed to provide detailed explanations for concepts or code snippets from a humanoid robotics textbook.
            Explain the following topic or code snippet clearly and concisely, using simple language where possible.
            If a code snippet is provided, analyze it and explain its functionality.
            
            Topic/Code:
            {topic_or_code}
            
            Context (if available):
            {context}
            
            Explanation:""",
            input_variables=["topic_or_code", "context"]
        )

    def generate_explanation(self, topic_or_code: str) -> Optional[str]:
        """
        Generates an explanation for a given topic or code snippet by retrieving relevant context
        and using an LLM.
        """
        if self.llm is None:
            print("LLM not initialized for ExplainAgent.")
            return None
        
        # Use RAGService to retrieve relevant context for the topic/code
        retrieved_response = self.rag_service.query(f"Provide context for: {topic_or_code}", n_results=3)
        
        context = retrieved_response 

        if "I couldn't find any relevant information" in context:
            context = "No specific context found in the textbook, providing a general explanation."
            # Optionally, we might still try to explain without specific context

        formatted_prompt = self.explain_prompt_template.format(topic_or_code=topic_or_code, context=context)
        
        try:
            explanation = self.llm.invoke(formatted_prompt)
            return explanation
        except Exception as e:
            print(f"Error generating explanation: {e}")
            print(f"LLM output: {explanation}")
            return None

if __name__ == "__main__":
    # Example usage
    # Make sure Ollama server is running and you have 'llama2' model pulled (ollama pull llama2) 
    
    explain_agent = ExplainAgent()
    
    if explain_agent.llm:
        print("ExplainAgent initialized. Generating explanation...")
        
        # Example 1: Explaining a topic
        explanation_topic = explain_agent.generate_explanation("The Zero Moment Point")
        if explanation_topic:
            print("\n--- Explanation for 'The Zero Moment Point' ---")
            print(explanation_topic)
        else:
            print("Failed to generate explanation for topic.")
            
        # Example 2: Explaining a code snippet
        code_snippet = """
def forward_kinematics(theta1, theta2, l1, l2):
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
    return x, y
        """
        explanation_code = explain_agent.generate_explanation(code_snippet)
        if explanation_code:
            print("\n--- Explanation for Code Snippet ---")
            print(explanation_code)
        else:
            print("Failed to generate explanation for code.")
    else:
        print("ExplainAgent could not be initialized. Please check your setup.")
