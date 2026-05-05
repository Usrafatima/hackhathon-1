import React, { useState, useRef, useEffect } from "react";
import axios from "axios";
import { Send, MessageSquare, X, Trash2, Bot, User } from "lucide-react";
import styles from "./module.css"; // Using CSS modules if possible, but the current file uses global-ish imports. 
import "./module.css";

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const messagesEndRef = useRef(null);

  const toggleChat = () => setIsOpen(!isOpen);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isTyping]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { type: "user", text: input, timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    const currentInput = input;
    setInput("");
    setIsTyping(true);

    try {
      const response = await axios.post("http://localhost:8000/api/v1/chat/book", {
        query: currentInput,
      });

      const botMessage = { 
        type: "bot", 
        text: response.data.response_text, 
        timestamp: new Date() 
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (err) {
      console.error("API Error:", err);
      let errorText = "The robotic brain is currently recalibrating. Please try again later.";
      
      if (err.response?.data?.response_text) {
        errorText = err.response.data.response_text;
      } else if (err.response?.data?.detail) {
        errorText = typeof err.response.data.detail === 'string' 
          ? err.response.data.detail 
          : "Context threshold not met.";
      }

      setMessages(prev => [...prev, { type: "bot", text: errorText, isError: true, timestamp: new Date() }]);
    } finally {
      setIsTyping(false);
    }
  };

  const clearChat = (e) => {
    e.stopPropagation();
    setMessages([]);
  };

  return (
    <div className={`chatbot-container ${isOpen ? "open" : ""}`}>
      {!isOpen ? (
        <button className="chatbot-launcher" onClick={toggleChat}>
          <MessageSquare size={28} />
        </button>
      ) : (
        <div className="chatbot-window">
          <div className="chatbot-header" onClick={toggleChat}>
            <div className="header-info">
              <Bot size={20} className="header-icon" />
              <span>BookBot Intelligence</span>
            </div>
            <div className="header-actions">
              <button className="action-btn" onClick={clearChat} title="Clear Conversation">
                <Trash2 size={16} />
              </button>
              <button className="action-btn" onClick={toggleChat}>
                <X size={18} />
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 && (
              <div className="welcome-msg">
                <Bot size={40} style={{ opacity: 0.2, marginBottom: '10px' }} />
                <p>Hello! Ask me anything about the Humanoid Robot Book.</p>
              </div>
            )}
            
            {messages.map((msg, index) => (
              <div key={index} className={`message-wrapper ${msg.type}`}>
                <div className="avatar">
                  {msg.type === "bot" ? <Bot size={14} /> : <User size={14} />}
                </div>
                <div className={`message-bubble ${msg.isError ? 'error' : ''}`}>
                  {msg.text}
                </div>
              </div>
            ))}
            
            {isTyping && (
              <div className="message-wrapper bot">
                <div className="avatar"><Bot size={14} /></div>
                <div className="message-bubble typing">
                  <span className="dot"></span>
                  <span className="dot"></span>
                  <span className="dot"></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-container">
            <input
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={e => e.key === "Enter" && sendMessage()}
              placeholder="Query the robot brain..."
              autoFocus
            />
            <button className="send-btn" onClick={sendMessage} disabled={!input.trim() || isTyping}>
              <Send size={18} />
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatBot;
