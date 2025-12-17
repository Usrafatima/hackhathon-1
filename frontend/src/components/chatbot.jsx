import React, { useState, useRef } from "react";
import axios from "axios";
import "../components/module.css"



const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(true); // Open by default
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const messagesEndRef = useRef(null);

  const toggleChat = () => setIsOpen(!isOpen);

  const sendMessage = async () => {
    if (!input.trim()) return;

    // User bubble (right side)
    const userMessage = { type: "user", text: input };
    setMessages(prev => [...prev, userMessage]);
    setInput("");

    // Scroll down
    setTimeout(() => {
      messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, 100);

    try {
      // Backend API call (replace YOUR_BACKEND_API_URL with your real API)
      const response = await axios.post("http://localhost:8000/api/v1/chat/book", {
        message: input,
      });

      // Bot bubble (left side)
      const botMessage = { type: "bot", text: response.data.reply };
      setMessages(prev => [...prev, botMessage]);

      setTimeout(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
      }, 100);
    } catch (err) {
      console.error(err);
      const botMessage = { type: "bot", text: "Sorry, backend se response nahi aaya." };
      setMessages(prev => [...prev, botMessage]);
    }
  };

  return (
    <div className={`chatbot-container ${isOpen ? "open" : ""}`}>
      <div className="chatbot-header" onClick={toggleChat}>
        Chat with BookBot 📚
      </div>

      {isOpen && (
        <div className="chatbot-body">
          <div className="chatbot-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.type}`}>
                {msg.text}
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input">
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={e => e.key === "Enter" && sendMessage()}
              placeholder="Type your message..."
            />
            <button onClick={sendMessage}>Send</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatBot;