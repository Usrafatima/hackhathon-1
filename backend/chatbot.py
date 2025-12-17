import cohere

# Replace with your working API key (or better: use environment variable)
co = cohere.Client("5lkNIoDgAJ9BYWL23IwpFrgWeg3yJ6QLqZuNE4my")  # Paste your key here for testing

# Optional: Keep conversation history for better responses
chat_history = []

print("Cohere Chatbot is ready! Type 'quit' to exit.\n")

while True:
    user_message = input("You: ")
    if user_message.lower() == "quit":
        print("Goodbye!")
        break
    
    # Send to Cohere
    response = co.chat(
    message=user_message,
    model="command-a-03-2025",  # New strongest model
    temperature=0.7,
    chat_history=chat_history
)
    
    bot_reply = response.text
    print("Bot:", bot_reply, "\n")
    
    # Update history so the bot remembers the conversation
    chat_history.append({"role": "USER", "message": user_message})
    chat_history.append({"role": "CHATBOT", "message": bot_reply})