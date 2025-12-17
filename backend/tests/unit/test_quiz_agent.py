import pytest
from unittest.mock import Mock, patch
from backend.src.agents.quiz_agent import QuizAgent
from backend.src.services.rag_service import RAGService # Required for patching

@pytest.fixture
def mock_rag_service_for_quiz():
    with patch('backend.src.agents.quiz_agent.RAGService') as MockRAGService:
        mock_instance = Mock()
        mock_instance.llm = Mock()
        mock_instance.llm.invoke.return_value = """
            [
                {
                    "question": "What is the capital of France?",
                    "options": ["Paris", "Berlin", "Rome", "Madrid"],
                    "correct_answer": "Paris"
                }
            ]
        """
        mock_instance.query.return_value = "Mocked context about the topic for quiz generation."
        MockRAGService.return_value = mock_instance
        yield mock_instance

@pytest.fixture
def quiz_agent_instance(mock_rag_service_for_quiz):
    agent = QuizAgent(model_name="test_model")
    return agent

def test_quiz_agent_initialization(quiz_agent_instance):
    assert quiz_agent_instance.rag_service is not None
    assert quiz_agent_instance.llm is not None

def test_generate_quiz_success(quiz_agent_instance, mock_rag_service_for_quiz):
    topic = "General Knowledge"
    num_questions = 1
    quiz = quiz_agent_instance.generate_quiz(topic, num_questions)

    assert quiz is not None
    assert isinstance(quiz, list)
    assert len(quiz) == 1
    assert quiz[0]['question'] == "What is the capital of France?"
    mock_rag_service_for_quiz.query.assert_called_once_with(f"Provide detailed information about: {topic}", n_results=5)
    quiz_agent_instance.llm.invoke.assert_called_once()

def test_generate_quiz_llm_not_initialized():
    # Simulate LLM not being initialized in RAGService
    with patch('backend.src.agents.quiz_agent.RAGService') as MockRAGService:
        mock_instance = Mock()
        mock_instance.llm = None
        MockRAGService.return_value = mock_instance
        agent = QuizAgent(model_name="test_model") # will try to initialize with None
        
        quiz = agent.generate_quiz("Test Topic")
        assert quiz is None

def test_generate_quiz_no_context_found(quiz_agent_instance, mock_rag_service_for_quiz):
    mock_rag_service_for_quiz.query.return_value = "I couldn't find any relevant information in the textbook to answer your question."
    quiz = quiz_agent_instance.generate_quiz("Non-existent Topic")
    assert quiz is None
    mock_rag_service_for_quiz.query.assert_called_once()

def test_generate_quiz_invalid_json_output(quiz_agent_instance, mock_rag_service_for_quiz):
    mock_rag_service_for_quiz.llm.invoke.return_value = "This is not valid JSON."
    quiz = quiz_agent_instance.generate_quiz("Topic")
    assert quiz is None
