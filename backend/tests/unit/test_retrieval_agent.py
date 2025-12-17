import pytest
from unittest.mock import MagicMock, patch
from backend.src.services.agents.retrieval_agent import QdrantRetrievalAgent
from backend.src.core.config import Settings

# Mock the settings to prevent actual API calls during testing
@pytest.fixture(autouse=True)
def mock_settings():
    with patch('backend.src.core.config.settings', new=Settings(
        COHERE_API_KEY="mock_cohere_key",
        QDRANT_API_URL="mock_qdrant_url",
        QDRANT_API_KEY="mock_qdrant_key",
        NEON_DATABASE_URL="mock_neon_url"
    )):
        yield

@pytest.fixture
def retrieval_agent():
    return QdrantRetrievalAgent()

@patch('cohere.Client')
@patch('qdrant_client.QdrantClient')
def test_retrieval_agent_execute(mock_qdrant_client, mock_cohere_client, retrieval_agent):
    # Mock Cohere client response
    mock_cohere_instance = mock_cohere_client.return_value
    mock_cohere_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

    # Mock Qdrant client response
    mock_qdrant_instance = mock_qdrant_client.return_value
    mock_search_result = MagicMock()
    mock_search_result.score = 0.95
    mock_search_result.payload = {"raw_text": "sample text", "chapter_number": 1}
    mock_qdrant_instance.search.return_value = [mock_search_result]

    query = "test query"
    top_k = 1

    result = retrieval_agent.execute(query, top_k)

    mock_cohere_instance.embed.assert_called_once_with(
        texts=[query],
        model="embed-english-v3.0",
        input_type="search_query"
    )
    mock_qdrant_instance.search.assert_called_once_with(
        collection_name="humanoid-robot-book",
        query_vector=[0.1, 0.2, 0.3],
        limit=top_k,
        with_payload=True
    )

    assert isinstance(result, list)
    assert len(result) == 1
    assert result[0]["score"] == 0.95
    assert result[0]["payload"]["raw_text"] == "sample text"
