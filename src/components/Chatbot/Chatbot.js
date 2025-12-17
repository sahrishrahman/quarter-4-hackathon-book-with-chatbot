import React, { useState, useRef, useEffect } from 'react';
import './Chatbot.css';

const Chatbot = ({ backendUrl = 'http://localhost:8000' }) => {
  const [messages, setMessages] = useState([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    if (selection) {
      return selection.toString().trim();
    }
    return '';
  };

  // Function to handle text selection with additional context
  const handleTextSelection = () => {
    const text = getSelectedText();
    if (text) {
      // Limit the selected text to a reasonable length to avoid token limits
      const limitedText = text.length > 500 ? text.substring(0, 500) + '...' : text;
      setSelectedText(limitedText);
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleSelection = () => {
      // Use setTimeout to get the selection after it's made
      setTimeout(() => {
        const text = getSelectedText();
        if (text) {
          // Limit the selected text to a reasonable length to avoid token limits
          const limitedText = text.length > 500 ? text.substring(0, 500) + '...' : text;
          setSelectedText(limitedText);
        }
      }, 0);
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Get any currently selected text from the page (this takes priority)
      const currentSelectedText = getSelectedText().trim();
      // Use current selection if available, otherwise use stored selection
      const contextText = currentSelectedText || selectedText;

      const response = await fetch(`${backendUrl}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputText,
          history: messages.map(msg => ({
            role: msg.sender,
            content: msg.text,
            timestamp: msg.timestamp
          })),
          selected_text: contextText ? contextText : null,
          document_ids: null,
          temperature: 0.7
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSelectedText('');
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Physical AI Book Assistant</h3>
        <button onClick={clearChat} className="clear-chat-btn">Clear</button>
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your Physical AI Book assistant.</p>
            <p>Ask me anything about the content in the book, or select text on the page to ask specific questions about it.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
            >
              <div className="message-content">
                {message.text}
              </div>
              {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  Sources: {message.sources.slice(0, 3).join(', ')}
                  {message.sources.length > 3 && ` +${message.sources.length - 3} more`}
                </div>
              )}
              <div className="message-timestamp">
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <strong>Selected text for context:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
          <button
            onClick={() => setSelectedText('')}
            className="remove-selection-btn"
            title="Remove selection"
          >
            ×
          </button>
        </div>
      )}

      <div className="chatbot-input-area">
        <textarea
          ref={inputRef}
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the Physical AI Book..."
          disabled={isLoading}
          rows="1"
        />
        <button
          onClick={sendMessage}
          disabled={!inputText.trim() || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default Chatbot;