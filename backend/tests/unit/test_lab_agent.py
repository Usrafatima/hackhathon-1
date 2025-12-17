import pytest
from unittest.mock import Mock, patch
from backend.src.agents.lab_agent import LabAgent
from backend.src.services.rag_service import RAGService # Required for patching

@pytest.fixture
def mock_rag_service_for_lab():
    with patch('backend.src.agents.lab_agent.RAGService') as MockRAGService:
        mock_instance = Mock()
        mock_instance.llm = Mock()
        mock_instance.llm.invoke.return_value = """
            {
                "title": "Build a Simple Robot Arm",
                "objective": "Understand basic motor control and inverse kinematics.",
                "materials": ["Arduino", "2x Servo Motors", "Cardboard"],
                "instructions": ["1. Assemble arm.", "2. Wire servos.", "3. Upload code."],
                "reasoning": "Practical application of kinematics.",
                "expected_outcome": "Working 2-DOF arm."
            }
        """
        mock_instance.query.return_value = "Mocked context about robotic arm for lab generation."
        MockRAGService.return_value = mock_instance
        yield mock_instance

@pytest.fixture
def lab_agent_instance(mock_rag_service_for_lab):
    agent = LabAgent(model_name="test_model")
    return agent

def test_lab_agent_initialization(lab_agent_instance):
    assert lab_agent_instance.rag_service is not None
    assert lab_agent_instance.llm is not None

def test_generate_lab_success(lab_agent_instance, mock_rag_service_for_lab):
    topic = "Robotic Arm Kinematics"
    lab = lab_agent_instance.generate_lab(topic)

    assert lab is not None
    assert isinstance(lab, dict)
    assert lab['title'] == "Build a Simple Robot Arm"
    mock_rag_service_for_lab.query.assert_called_once_with(f"Provide detailed information about: {topic} for a lab exercise.", n_results=5)
    lab_agent_instance.llm.invoke.assert_called_once()

def test_generate_lab_llm_not_initialized():
    with patch('backend.src.agents.lab_agent.RAGService') as MockRAGService:
        mock_instance = Mock()
        mock_instance.llm = None
        MockRAGService.return_value = mock_instance
        agent = LabAgent(model_name="test_model")
        
        lab = agent.generate_lab("Test Topic")
        assert lab is None

def test_generate_lab_no_context_found(lab_agent_instance, mock_rag_service_for_lab):
    mock_rag_service_for_lab.query.return_value = "I couldn't find any relevant information in the textbook to answer your question."
    lab = lab_agent_instance.generate_lab("Non-existent Lab Topic")
    assert lab is None
    mock_rag_service_for_lab.query.assert_called_once()

def test_generate_lab_invalid_json_output(lab_agent_instance, mock_rag_service_for_lab):
    mock_rag_service_for_lab.llm.invoke.return_value = "This is not valid JSON."
    lab = lab_agent_instance.generate_lab("Topic")
    assert lab is None
