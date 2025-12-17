from abc import ABC, abstractmethod
from typing import List, Dict, Any

class BaseAgent(ABC):
    @abstractmethod
    def execute(self, *args, **kwargs) -> Any:
        pass

class RetrievalAgent(BaseAgent):
    @abstractmethod
    def execute(self, query: str, top_k: int) -> List[Dict[str, Any]]:
        """
        Embeds the query and retrieves the top_k most relevant documents.
        Returns a list of documents with their content and metadata.
        """
        pass

class ContextValidationAgent(BaseAgent):
    @abstractmethod
    def execute(self, query: str, context: List[Dict[str, Any]]) -> str:
        """
        Validates the retrieved context and formats it for the generation agent.
        Returns a string of formatted context.
        """
        pass

class AnswerGenerationAgent(BaseAgent):
    @abstractmethod
    def execute(self, query: str, context: str) -> str:
        """
        Generates an answer based on the query and the provided context.
        Returns the generated answer as a string.
        """
        pass
