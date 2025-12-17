import os
from typing import List, Dict, Optional
from langchain_community.llms import Ollama
from langchain_core.prompts import PromptTemplate
from sentence_transformers import SentenceTransformer
import json # Added for JSON parsing

# Assuming RAGService can be imported or its components re-used
from src.services.rag_service import RAGOrchestrator # Re-using RAGService for context retrieval

class QuizAgent:
    def __init__(self, model_name="llama2", embedding_model_name="all-MiniLM-L6-v2", chroma_path=None):
        self.rag_service = RAGOrchestrator()
        self.llm = Ollama(model=model_name, timeout=60) # Assuming Ollama is used for the LLM

        self.quiz_prompt_template = PromptTemplate(
            template="""You are an AI assistant designed to create multiple-choice questions (MCQs) for a humanoid robotics textbook.
            Generate {num_questions} MCQs based on the following context.
            Each MCQ should have 4 options, with only one correct answer.
            Provide the correct answer for each question.
            
            Context:
            {context}
            
            Format your output as a JSON list of questions, where each question is an object with 'question', 'options' (list of strings), and 'correct_answer' (string).
            
            Example:
            [
                {{
                    "question": "What is an actuator?",
                    "options": ["A device that senses the environment", "A device that provides motion", "A device that stores energy", "A device that processes data"],
                    "correct_answer": "A device that provides motion"
                }}
            ]
            
            Questions:""",
            input_variables=["context", "num_questions"]
        )

    def generate_quiz(self, topic: str, num_questions: int = 3) -> Optional[List[Dict]]:
        """
        Generates MCQs based on a given topic by retrieving relevant context
        and using an LLM.
        """
        if self.llm is None:
            print("LLM not initialized for QuizAgent.")
            return None
        
        # Use RAGOrchestrator to retrieve relevant context for the topic
        orchestrator_response = self.rag_service.execute_book_wide_chat(f"Provide detailed information about: {topic}")
        
        # Extract context from the retrieved_response
        retrieved_docs = orchestrator_response.get("retrieved_context", [])

        context = ""
        for doc in retrieved_docs:
            context += doc['payload']['raw_text'] + "\n\n"

        if not context.strip():
            print(f"No relevant context found for topic: {topic}")
            return None

        formatted_prompt = self.quiz_prompt_template.format(context=context, num_questions=num_questions)
        
        try:
            quiz_json_str = self.llm.invoke(formatted_prompt)
            # LLMs can sometimes generate extra text. Try to parse JSON strictly.
            # Find the first and last bracket to ensure valid JSON
            start_index = quiz_json_str.find('[')
            end_index = quiz_json_str.rfind(']')
            if start_index != -1 and end_index != -1:
                quiz_json_str = quiz_json_str[start_index : end_index + 1]
            
            quiz = json.loads(quiz_json_str)
            return quiz
        except Exception as e:
            print(f"Error generating or parsing quiz: {e}")
            print(f"LLM output: {quiz_json_str}")
            return None

if __name__ == "__main__":
    # Example usage
    # Make sure Ollama server is running and you have 'llama2' model pulled (ollama pull llama2)
    import json # Redundant import here, already at top
    
    quiz_agent = QuizAgent()
    
    if quiz_agent.llm:
        print("QuizAgent initialized. Generating quiz...")
        quiz_data = quiz_agent.generate_quiz("Robot Kinematics", num_questions=2)
        
        if quiz_data:
            print("\n--- Generated Quiz ---")
            print(json.dumps(quiz_data, indent=2))
        else:
            print("Failed to generate quiz.")
    else:
        print("QuizAgent could not be initialized. Please check your setup.")
