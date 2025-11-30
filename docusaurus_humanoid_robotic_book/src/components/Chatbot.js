// import React, { useState, useRef, useEffect } from 'react';
// import clsx from 'clsx';
// import styles from './Chatbot.module.css';

// const Chatbot = () => {
//   const [isOpen, setIsOpen] = useState(false);
//   const [messages, setMessages] = useState([
//     {
//       id: 1,
//       text: "Hi! I'm your documentation assistant. How can I help you today?",
//       sender: 'bot',
//       timestamp: new Date(),
//     }
//   ]);
//   const [inputValue, setInputValue] = useState('');
//   const [isTyping, setIsTyping] = useState(false);
//   const messagesEndRef = useRef(null);

//   const scrollToBottom = () => {
//     messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
//   };

//   useEffect(() => {
//     scrollToBottom();
//   }, [messages]);

//   const handleSendMessage = () => {
//     if (inputValue.trim() === '') return;

//     // Add user message
//     const userMessage = {
//       id: messages.length + 1,
//       text: inputValue,
//       sender: 'user',
//       timestamp: new Date(),
//     };

//     setMessages(prev => [...prev, userMessage]);
//     setInputValue('');
//     setIsTyping(true);

//     // Simulate bot response after a delay
//     setTimeout(() => {
//       const botResponse = generateBotResponse(inputValue);
//       setMessages(prev => [...prev, botResponse]);
//       setIsTyping(false);
//     }, 1000);
//   };

//   const generateBotResponse = (userInput) => {
//     const input = userInput.toLowerCase();
    
//     // Simple response logic - you can expand this or connect to an API
//     if (input.includes('hello') || input.includes('hi') || input.includes('hey')) {
//       return {
//         id: messages.length + 2,
//         text: "Hello! How can I assist you with our documentation today?",
//         sender: 'bot',
//         timestamp: new Date(),
//       };
//     } else if (input.includes('thank')) {
//       return {
//         id: messages.length + 2,
//         text: "You're welcome! Is there anything else I can help you with?",
//         sender: 'bot',
//         timestamp: new Date(),
//       };
//     } else if (input.includes('documentation') || input.includes('docs')) {
//       return {
//         id: messages.length + 2,
//         text: "I can help you navigate our documentation. What specific topic are you looking for?",
//         sender: 'bot',
//         timestamp: new Date(),
//       };
//     } else {
//       return {
//         id: messages.length + 2,
//         text: "I'm still learning! For now, I suggest browsing our documentation or asking in our community forums for more complex questions.",
//         sender: 'bot',
//         timestamp: new Date(),
//       };
//     }
//   };

//   const handleKeyPress = (e) => {
//     if (e.key === 'Enter' && !e.shiftKey) {
//       e.preventDefault();
//       handleSendMessage();
//     }
//   };

//   const toggleChat = () => {
//     setIsOpen(!isOpen);
//   };

//   const formatTime = (date) => {
//     return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
//   };

//   return (
//     <div className={styles.chatbotContainer}>
//       {/* Chat Button */}
//       <button 
//         className={clsx(styles.chatButton, { [styles.hidden]: isOpen })}
//         onClick={toggleChat}
//       >
//         <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
//           <path d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z" fill="currentColor"/>
//         </svg>
//         <span>Need help?</span>
//       </button>

//       {/* Chat Window */}
//       {isOpen && (
//         <div className={styles.chatWindow}>
//           <div className={styles.chatHeader}>
//             <div className={styles.botInfo}>
//               <div className={styles.botAvatar}>
//                 <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
//                   <path d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z" fill="white"/>
//                 </svg>
//               </div>
//               <div>
//                 <h3>Documentation Assistant</h3>
//                 <span className={styles.status}>
//                   <span className={styles.statusIndicator}></span>
//                   Online
//                 </span>
//               </div>
//             </div>
//             <button className={styles.closeButton} onClick={toggleChat}>
//               <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
//                 <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
//               </svg>
//             </button>
//           </div>

//           <div className={styles.messagesContainer}>
//             {messages.map((message) => (
//               <div
//                 key={message.id}
//                 className={clsx(
//                   styles.message,
//                   message.sender === 'user' ? styles.userMessage : styles.botMessage
//                 )}
//               >
//                 <div className={styles.messageContent}>
//                   <p>{message.text}</p>
//                   <span className={styles.timestamp}>
//                     {formatTime(message.timestamp)}
//                   </span>
//                 </div>
//               </div>
//             ))}
//             {isTyping && (
//               <div className={clsx(styles.message, styles.botMessage)}>
//                 <div className={styles.typingIndicator}>
//                   <span></span>
//                   <span></span>
//                   <span></span>
//                 </div>
//               </div>
//             )}
//             <div ref={messagesEndRef} />
//           </div>

//           <div className={styles.inputContainer}>
//             <div className={styles.inputWrapper}>
//               <textarea
//                 value={inputValue}
//                 onChange={(e) => setInputValue(e.target.value)}
//                 onKeyPress={handleKeyPress}
//                 placeholder="Type your message here..."
//                 rows="1"
//                 className={styles.textInput}
//               />
//               <button 
//                 onClick={handleSendMessage}
//                 disabled={inputValue.trim() === ''}
//                 className={styles.sendButton}
//               >
//                 <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
//                   <path d="M22 2L11 13M22 2L15 22L11 13M22 2L2 9L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
//                 </svg>
//               </button>
//             </div>
//           </div>
//         </div>
//       )}
//     </div>
//   );
// };

// export default Chatbot;

import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Robotics and AI textbook assistant. I can help you find information from all your modules. What would you like to learn about?",
      sender: 'bot',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const [userPreference, setUserPreference] = useState('english');
  const messagesEndRef = useRef(null);

  // Backend API URL - adjust according to your deployment
  const BACKEND_URL = 'https://ai-book-backend-8qpa.onrender.com/'; // Change to your backend URL

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Backend API call function
  const sendMessageToBackend = async (message) => {
    try {
      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: message
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data.response;
    } catch (error) {
      console.error('Error calling backend API:', error);
      return "I'm having trouble connecting to the textbook knowledge base. Please try again later.";
    }
  };

  const handleSendMessage = async () => {
    if (inputValue.trim() === '') return;

    const userMessage = {
      id: messages.length + 1,
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsTyping(true);

    const userInput = inputValue.toLowerCase();
    
    // Handle language switching locally
    if (userInput.includes('roman urdu') || userInput.includes('urdu mein')) {
      setUserPreference('urdu');
      setTimeout(() => {
        const botResponse = {
          id: messages.length + 2,
          text: "Theek hai, main ab Roman Urdu mein jawab doonga. Aap robotics ke bare mein kya janna chahte hain?",
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botResponse]);
        setIsTyping(false);
      }, 1000);
      return;
    }
    
    if (userInput.includes('english') || userInput.includes('angrezi')) {
      setUserPreference('english');
      setTimeout(() => {
        const botResponse = {
          id: messages.length + 2,
          text: "Okay, I'll respond in English from now on. What would you like to know about robotics?",
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botResponse]);
        setIsTyping(false);
      }, 1000);
      return;
    }

    // For all other queries, call the backend
    try {
      const botResponseText = await sendMessageToBackend(inputValue);
      
      const botResponse = {
        id: messages.length + 2,
        text: botResponseText,
        sender: 'bot',
        timestamp: new Date(),
      };
      
      setMessages(prev => [...prev, botResponse]);
    } catch (error) {
      console.error('Error in handleSendMessage:', error);
      const errorResponse = {
        id: messages.length + 2,
        text: "Sorry, I'm having trouble accessing the textbook content right now. Please try again later.",
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorResponse]);
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  // Quick suggestion buttons
  const quickSuggestions = [
    "What is Robotic Nervous System?",
    "Explain Gazebo simulation",
    "Tell me about NVIDIA Isaac",
    "What is VLA in robotics?"
  ];

  const handleQuickSuggestion = (suggestion) => {
    setInputValue(suggestion);
  };

  return (
    <div className={styles.chatbotContainer}>
      <button 
        className={clsx(styles.chatButton, { [styles.hidden]: isOpen })}
        onClick={toggleChat}
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z" fill="currentColor"/>
        </svg>
        <span>Textbook Assistant</span>
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.botInfo}>
              <div className={styles.botAvatar}>📚</div>
              <div>
                <h3>Robotics Textbook Expert</h3>
                <span className={styles.status}>
                  <span className={styles.statusIndicator}></span>
                  {userPreference === 'english' ? 'Online - Powered by AI' : 'Roman Urdu Mode'}
                </span>
              </div>
            </div>
            <button className={styles.closeButton} onClick={toggleChat}>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  styles.message,
                  message.sender === 'user' ? styles.userMessage : styles.botMessage
                )}
              >
                <div className={styles.messageContent}>
                  <p style={{whiteSpace: 'pre-line'}}>{message.text}</p>
                  <span className={styles.timestamp}>
                    {formatTime(message.timestamp)}
                  </span>
                </div>
              </div>
            ))}
            {isTyping && (
              <div className={clsx(styles.message, styles.botMessage)}>
                <div className={styles.typingIndicator}>
                  <span>Searching textbook...</span>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Quick Suggestions */}
          {messages.length <= 2 && (
            <div className={styles.quickSuggestions}>
              <p>Try asking about:</p>
              <div className={styles.suggestionButtons}>
                {quickSuggestions.map((suggestion, index) => (
                  <button
                    key={index}
                    className={styles.suggestionButton}
                    onClick={() => handleQuickSuggestion(suggestion)}
                  >
                    {suggestion}
                  </button>
                ))}
              </div>
            </div>
          )}

          <div className={styles.inputContainer}>
            <div className={styles.inputWrapper}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask anything about Robotics and AI textbook..."
                rows="1"
                className={styles.textInput}
              />
              <button 
                onClick={handleSendMessage}
                disabled={inputValue.trim() === '' || isTyping}
                className={styles.sendButton}
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 2L11 13M22 2L15 22L11 13M22 2L2 9L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
            <div className={styles.inputHint}>
              Powered by actual textbook content from all modules
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;
