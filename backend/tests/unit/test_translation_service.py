import pytest
from backend.src.services.translation_service import TranslationService

@pytest.fixture
def translation_service_instance():
    return TranslationService()

def test_translation_to_urdu(translation_service_instance):
    text = "Hello World"
    translated_text = translation_service_instance.translate(text, "ur")
    assert "[URDU Translation of: Hello World]" == translated_text

def test_translation_to_english(translation_service_instance):
    text = "Hello World"
    translated_text = translation_service_instance.translate(text, "en")
    assert text == translated_text

def test_translation_unsupported_language(translation_service_instance):
    text = "Hello World"
    translated_text = translation_service_instance.translate(text, "fr")
    assert "[FR Translation not supported for: Hello World]" == translated_text
