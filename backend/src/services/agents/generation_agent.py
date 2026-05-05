from huggingface_hub import InferenceClient
from typing import Tuple
from src.core.config import settings

class HuggingFaceGenerationAgent:
    def __init__(self):
        self.client = InferenceClient(token=settings.HUGGINGFACE_API_TOKEN)
        # Using a recommended instruction-following model
        self.model = "mistralai/Mistral-7B-Instruct-v0.2"

    def execute(self, query: str, formatted_context: str) -> Tuple[str, str]:
        """
        Generates an answer using the Hugging Face Inference API.
        """
        system_prompt = (
            "You are an expert assistant for the 'Humanoid Robot Book'. "
            "Your task is to answer the user's question based *only* on the provided context document. "
            "If the context does not contain the information needed to answer the question, you must state: "
            "'I'm sorry, but I can't answer that based on the available context from the book.' "
            "Do not use any external knowledge or make up information."
        )

        user_prompt = (
            f"CONTEXT:\n{formatted_context}\n\n"
            f"USER'S QUESTION:\n{query}"
        )
        
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ]
        
        llm_prompt_used = f"System: {system_prompt}\nUser: {user_prompt}"

        try:
            # Use the chat_completion method for conversational models
            response = self.client.chat_completion(
                messages=messages,
                model=self.model,
                max_tokens=1024,
                temperature=0.1,
            )
            
            final_response_text = response.choices[0].message.content.strip()
            return final_response_text, llm_prompt_used

        except Exception as e:
            print(f"Error in Hugging Face generation: {e}")
            return "Sorry, I encountered an issue while generating a response.", llm_prompt_used