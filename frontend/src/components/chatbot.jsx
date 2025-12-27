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
        query: input, // Changed from 'message' to 'query'
      });

      // Bot bubble (left side) - corrected to use `response_text`
      const botMessage = { type: "bot", text: response.data.response_text };
      setMessages(prev => [...prev, botMessage]);

      setTimeout(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
      }, 100);
    } catch (err) {
      console.error("API Error:", err);
      let errorText = "Sorry, an unexpected error occurred.";

      if (err.response?.data) {
        const { detail, response_text } = err.response.data;

        if (typeof response_text === 'string' && response_text) {
          errorText = response_text;
        } else if (detail) {
          if (Array.isArray(detail) && detail[0]?.msg) {
            // Handle FastAPI validation error array
            errorText = `Error: ${detail[0].msg} (in ${detail[0].loc.join(" > ")})`;
          } else if (typeof detail === 'string') {
            errorText = detail;
          } else if (typeof detail === 'object' && detail !== null) {
            errorText = "Received a complex error object from the server.";
          }
        }
      }
      
      const botMessage = { type: "bot", text: errorText };
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