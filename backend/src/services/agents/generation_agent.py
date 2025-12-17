from typing import Tuple
from cohere import Client  # Make sure this is correct — adjust if needed

class CohereAnswerGenerationAgent:
    def __init__(self, cohere_api_key: str = None):
        # Initialize the Cohere client
        self.client = Client(cohere_api_key or "your-api-key-here")  # Or however you pass the key

    def execute(self, query: str, formatted_context: str) -> Tuple[str, str]:
        """
        Generates an answer using Cohere's new Chat API.
        """
        # Extract individual documents from the formatted context
        doc_blocks = [block.strip() for block in formatted_context.split("----------------------------------------") if block.strip()]
        documents = []
        for block in doc_blocks:
            lines = block.split("\n")
            content_lines = []
            for line in lines:
                if line.startswith("--- Document"):
                    continue  # Skip the header
                content_lines.append(line)
            raw_text = "\n".join(content_lines).strip()
            if raw_text:
                documents.append({"content": raw_text})

        try:
            response = self.client.chat(
                model="command-r-plus",  # or "command-r", "command", etc. — use what you have access to
                message=query,
                documents=documents,
                temperature=0.3,
                max_tokens=512,
            )

            final_response_text = response.text
            llm_prompt_used = "Cohere Chat API - prompt not exposed"

            return final_response_text, llm_prompt_used

        except Exception as e:
            print(f"Error in Cohere generation: {e}")
            return "Sorry, I encountered an issue while generating a response.", ""