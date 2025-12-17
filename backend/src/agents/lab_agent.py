import os
from typing import List, Dict, Optional
from langchain_community.llms import Ollama
from langchain_core.prompts import PromptTemplate

from backend.src.db.vector_db import get_chroma_client, get_or_create_collection
from backend.src.services.rag_service import RAGService # Re-using RAGService for context retrieval
import json

class LabAgent:
    def __init__(self, model_name="llama2", embedding_model_name="all-MiniLM-L6-v2", chroma_path=None):
        self.rag_service = RAGService(model_name=model_name, embedding_model_name=embedding_model_name, chroma_path=chroma_path)
        self.llm = self.rag_service.llm # Use the same LLM instance from RAGService

        self.lab_prompt_template = PromptTemplate(
            template='''You are an AI assistant designed to create hands-on lab instructions for a humanoid robotics textbook.
            Generate a detailed lab exercise based on the following context and topic: "{topic}".
            The lab should include:
            1. Lab Title
            2. Objective
            3. Materials/Software Required
            4. Step-by-step Instructions
            5. Reasoning (why this lab is important)
            6. Expected Outcome
            
            Context:
            {context}
            
            Format your output as a JSON object with keys: 'title', 'objective', 'materials', 'instructions' (list of strings), 'reasoning', 'expected_outcome'.
            
            Example:
            {{
                "title": "Simulating a Robotic Arm",
                "objective": "Understand basic kinematics.",
                "materials": ["RoboDK simulator"],
                "instructions": ["1. Open simulator.", "2. Load arm model.", "3. Move joints."],
                "reasoning": "To grasp joint-link relationship.",
                "expected_outcome": "Intuitive understanding of motion."
            }}
            
            Lab Exercise:''',
            input_variables=["context", "topic"]
        )

    def generate_lab(self, topic: str) -> Optional[Dict]:
        """
        Generates lab instructions based on a given topic by retrieving relevant context
        and using an LLM.
        """
        if self.llm is None:
            print("LLM not initialized for LabAgent.")
            return None
        
        # Use RAGService to retrieve relevant context for the topic
        retrieved_response = self.rag_service.query(f"Provide detailed information about: {topic} for a lab exercise.", n_results=5)
        
        context = retrieved_response 

        if "I couldn't find any relevant information" in context:
            print(f"No relevant context found for topic: {topic}")
            return None

        formatted_prompt = self.lab_prompt_template.format(context=context, topic=topic)
        
        try:
            lab_json_str = self.llm.invoke(formatted_prompt)
            # LLMs can sometimes generate extra text. Try to parse JSON strictly.
            start_index = lab_json_str.find('{')
            end_index = lab_json_str.rfind('}')
            if start_index != -1 and end_index != -1:
                lab_json_str = lab_json_str[start_index : end_index + 1]
            
            lab_data = json.loads(lab_json_str)
            return lab_data
        except Exception as e:
            print(f"Error generating or parsing lab: {e}")
            print(f"LLM output: {lab_json_str}")
            return None

if __name__ == "__main__":
    # Example usage
    # Make sure Ollama server is running and you have 'llama2' model pulled (ollama pull llama2)
    
    lab_agent = LabAgent()
    
    if lab_agent.llm:
        print("LabAgent initialized. Generating lab...")
        lab_data = lab_agent.generate_lab("Robot Kinematics Lab")
        
        if lab_data:
            print("\n--- Generated Lab ---")
            print(json.dumps(lab_data, indent=2))
        else:
            print("Failed to generate lab.")
    else:
        print("LabAgent could not be initialized. Please check your setup.")
