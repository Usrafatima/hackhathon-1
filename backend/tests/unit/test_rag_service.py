import pytest
from unittest.mock import Mock, patch
from backend.src.services.rag_service import RAGService
from backend.src.models.user_profile import UserProfile

# Mock the SentenceTransformer and Ollama to prevent actual model loading during tests
@pytest.fixture
def mock_embedding_model():
    with patch('backend.src.services.rag_service.SentenceTransformer') as MockSentenceTransformer:
        mock_instance = Mock()
        mock_instance.encode.return_value = [0.1, 0.2, 0.3] # Dummy embedding
        MockSentenceTransformer.return_value = mock_instance
        yield MockSentenceTransformer

@pytest.fixture
def mock_ollama_llm():
    with patch('backend.src.services.rag_service.Ollama') as MockOllama:
        mock_instance = Mock()
        mock_instance.invoke.return_value = "Mocked LLM response."
        MockOllama.return_value = mock_instance
        yield MockOllama

# Mock ChromaDB client and collection
@pytest.fixture
def mock_chromadb():
    with patch('backend.src.db.vector_db.get_chroma_client') as MockGetClient:
        with patch('backend.src.db.vector_db.get_or_create_collection') as MockGetOrCreateCollection:
            mock_collection = Mock()
            mock_collection.query.return_value = {
                'documents': [['Context document 1', 'Context document 2']],
                'metadatas': [[{'chapter_number': 1, 'title': 'Intro', 'section_title': 'Section A'}, {'chapter_number': 1, 'title': 'Intro', 'section_title': 'Section B'}]],
                'ids': [['id1', 'id2']]
            }
            mock_collection.count.return_value = 2
            MockGetOrCreateCollection.return_value = mock_collection
            yield MockGetClient, MockGetOrCreateCollection

@pytest.fixture
def rag_service_instance(mock_embedding_model, mock_ollama_llm, mock_chromadb):
    # Initialize RAGService with mocks
    service = RAGService(model_name="test_model", embedding_model_name="test_embedding", chroma_path="/tmp/test_chroma")
    service.llm = mock_ollama_llm.return_value # Ensure LLM is set to the mock instance
    return service

def test_rag_service_initialization(rag_service_instance):
    assert rag_service_instance.llm is not None
    assert rag_service_instance.embedding_model is not None
    assert rag_service_instance.collection is not None

def test_rag_service_query_basic(rag_service_instance):
    question = "What is a test question?"
    response = rag_service_instance.query(question)
    assert "Mocked LLM response." in response
    rag_service_instance.collection.query.assert_called_once()
    rag_service_instance.llm.invoke.assert_called_once()

def test_rag_service_query_with_user_profile(rag_service_instance):
    profile = UserProfile(username="test", preferred_language="en", reading_level="advanced", interests=["AI"])
    question = "Advanced topic?"
    response = rag_service_instance.query(question, user_profile=profile)
    assert "Mocked LLM response." in response
    rag_service_instance.llm.invoke.assert_called_once()
    args, kwargs = rag_service_instance.llm.invoke.call_args
    assert "Their reading level is advanced" in args[0]
    assert "Tailor your answer to a advanced level" in args[0]
    assert "Respond in en" in args[0]

def test_rag_service_query_no_documents_found(rag_service_instance):
    # Configure mock ChromaDB to return no documents
    rag_service_instance.collection.query.return_value = {
        'documents': [[]],
        'metadatas': [[]],
        'ids': [[]]
    }
    question = "Irrelevant question?"
    response = rag_service_instance.query(question)
    assert "I couldn't find any relevant information" in response
    rag_service_instance.llm.invoke.assert_not_called()

def test_rag_service_query_llm_not_initialized():
    service = RAGService(model_name="non_existent_model") # This will set llm to None
    service.llm = None # Explicitly ensure it's None for this test
    question = "Any question"
    response = service.query(question)
    assert "LLM is not initialized" in response

