import pytest
from unittest.mock import Mock, patch
from backend.src.agents.explain_agent import ExplainAgent
from backend.src.services.rag_service import RAGService # Required for patching

@pytest.fixture
def mock_rag_service_for_explain():
    with patch('backend.src.agents.explain_agent.RAGService') as MockRAGService:
        mock_instance = Mock()
        mock_instance.llm = Mock()
        mock_instance.llm.invoke.return_value = "This is a detailed explanation of the topic."
        mock_instance.query.return_value = "Mocked context about the topic for explanation."
        MockRAGService.return_value = mock_instance
        yield mock_instance

@pytest.fixture
def explain_agent_instance(mock_rag_service_for_explain):
    agent = ExplainAgent(model_name="test_model")
    return agent

def test_explain_agent_initialization(explain_agent_instance):
    assert explain_agent_instance.rag_service is not None
    assert explain_agent_instance.llm is not None

def test_generate_explanation_success(explain_agent_instance, mock_rag_service_for_explain):
    topic = "The Zero Moment Point"
    explanation = explain_agent_instance.generate_explanation(topic)

    assert explanation is not None
    assert "detailed explanation" in explanation
    mock_rag_service_for_explain.query.assert_called_once_with(f"Provide context for: {topic}", n_results=3)
    explain_agent_instance.llm.invoke.assert_called_once()

def test_generate_explanation_llm_not_initialized():
    with patch('backend.src.agents.explain_agent.RAGService') as MockRAGService:
        mock_instance = Mock()
        mock_instance.llm = None
        MockRAGService.return_value = mock_instance
        agent = ExplainAgent(model_name="test_model")
        
        explanation = agent.generate_explanation("Test Topic")
        assert explanation is None

def test_generate_explanation_no_context_found(explain_agent_instance, mock_rag_service_for_explain):
    # This test now expects a general explanation even without specific context
    mock_rag_service_for_explain.query.return_value = "I couldn't find any relevant information in the textbook to answer your question."
    explanation = explain_agent_instance.generate_explanation("Non-existent Topic")
    assert explanation is not None # Should still attempt a general explanation
    assert "detailed explanation" in explanation # Based on mock LLM invoke
    mock_rag_service_for_explain.query.assert_called_once()

