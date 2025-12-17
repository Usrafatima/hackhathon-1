from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_create_user():
    response = client.post(
        "/register",
        json={"username": "testuser", "password": "testpassword"},
    )
    assert response.status_code == 200
    assert response.json()["username"] == "testuser"
    assert "id" in response.json()
    assert "is_active" in response.json()

def test_login_for_access_token():
    # First, create a user to login with
    client.post(
        "/register",
        json={"username": "testuser2", "password": "testpassword2"},
    )
    
    response = client.post(
        "/token",
        data={"username": "testuser2", "password": "testpassword2"},
    )
    assert response.status_code == 200
    assert "access_token" in response.json()
    assert response.json()["token_type"] == "bearer"
