import pytest
from httpx import AsyncClient
from main import app # Assuming main.py is directly importable for testing
from unittest.mock import patch

# Mock the settings to prevent actual API calls during testing
@pytest.fixture(autouse=True)
def mock_settings_fixture():
    with patch('backend.src.core.config.settings') as mock_settings:
        mock_settings.COHERE_API_KEY = "mock_cohere_key"
        mock_settings.QDRANT_API_URL = "mock_qdrant_url"
        mock_settings.QDRANT_API_KEY = "mock_qdrant_key"
        mock_settings.NEON_DATABASE_URL = "mock_neon_url"
        yield

@pytest.fixture(name="client")
async def client_fixture():
    async with AsyncClient(app=app, base_url="http://test") as client:
        yield client

@pytest.mark.asyncio
async def test_chat_book_endpoint(client: AsyncClient):
    # Mock RAGOrchestrator to avoid external dependencies
    with patch('backend.src.services.rag_service.RAGOrchestrator') as MockOrchestrator:
        mock_instance = MockOrchestrator.return_value
        mock_instance.execute_book_wide_chat.return_value = {
            "response_text": "Mocked response for book chat.",
            "sources": [],
            "retrieved_context": [],
            "llm_prompt": "mock prompt",
            "is_refusal": False
        }

        response = await client.post(
            "/api/v1/chat/book",
            json={"query": "What is the capital of France?"}
        )
        assert response.status_code == 200
        assert response.json()["response_text"] == "Mocked response for book chat."

@pytest.mark.asyncio
async def test_chat_selection_endpoint(client: AsyncClient):
    # Mock RAGOrchestrator to avoid external dependencies
    with patch('backend.src.services.rag_service.RAGOrchestrator') as MockOrchestrator:
        mock_instance = MockOrchestrator.return_value
        mock_instance.execute_selection_chat.return_value = {
            "response_text": "Mocked response for selection chat.",
            "sources": [],
            "retrieved_context": [],
            "llm_prompt": "mock prompt",
            "is_refusal": False
        }

        response = await client.post(
            "/api/v1/chat/selection",
            json={"query": "Explain this.", "selected_text": "Some text."}
        )
        assert response.status_code == 200
        assert response.json()["response_text"] == "Mocked response for selection chat."

@pytest.mark.asyncio
async def test_ingest_endpoint(client: AsyncClient):
    # Mock the background task execution for simplicity
    with patch('backend.src.api.endpoints._run_ingestion_process') as mock_ingest_process:
        response = await client.post("/api/v1/ingest")
        assert response.status_code == 202
        assert response.json()["message"] == "Ingestion process started in the background."
        mock_ingest_process.assert_called_once()