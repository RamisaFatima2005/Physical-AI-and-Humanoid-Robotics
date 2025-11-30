
import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Robotics and AI documentation assistant. I can help you find information from all your modules. What would you like to learn about?",
      sender: 'bot',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const [userPreference, setUserPreference] = useState('english');
  const messagesEndRef = useRef(null);

  // Smart content database based on your documentation structure
  const smartContent = {
    'module1': {
      title: "Module 1: The Robotic Nervous System",
      description: "This module covers the fundamental architecture of robotic nervous systems that mimic biological systems for efficient robot operation.",
      chapters: {
        'chapter1': {
          title: "Chapter 1: Introduction to Robotic Nervous System",
          content: `The Robotic Nervous System consists of three main components:

🧠 SENSORY SYSTEM (Input)
• Cameras, LIDAR, ultrasonic sensors
• Touch and proximity sensors
• Environmental data collection

⚡ PROCESSING UNIT (Brain)
• Central decision-making system
• AI algorithms and neural networks
• Real-time data processing

🏃 MOTOR SYSTEM (Output)
• Actuators and motors
• Movement and manipulation
• Physical interaction

This architecture enables robots to perceive, process, and act in their environment efficiently.`
        }
      },
      keywords: ['sensory', 'processing', 'motor', 'nervous system', 'robotic architecture']
    },
    'module2': {
      title: "Module 2: The Digital Twin - Gazebo & Unity",
      description: "Learn about simulation environments and digital twin technology for robotics development.",
      chapters: {
        'chapter2': {
          title: "Chapter 2: Gazebo Simulation Environment",
          content: `Gazebo provides a comprehensive 3D simulation environment:

🖥️ 3D ROBOT SIMULATION
• Realistic physics engine
• Robot modeling and design
• Sensor simulation
• Environment creation

🔧 KEY FEATURES
• Multiple robot support
• Realistic sensor data
• Plugin system
• ROS integration

Gazebo allows testing and validation of robotic systems in safe virtual environments.`
        },
        'chapter3': {
          title: "Chapter 3: Unity Integration for Digital Twins",
          content: `Unity brings advanced visualization to robotics:

🎮 ADVANCED VISUALIZATION
• High-quality 3D graphics
• Real-time rendering
• VR/AR integration
• Interactive interfaces

🔄 DIGITAL TWIN CAPABILITIES
• Real-world synchronization
• Data visualization
• Predictive analytics
• Multi-platform deployment

Unity enhances simulation with photorealistic visuals and interactive experiences.`
        }
      },
      keywords: ['gazebo', 'unity', 'simulation', 'digital twin', '3d environment']
    },
    'module3': {
      title: "Module 3: The AI Robot Brain - NVIDIA Isaac™",
      description: "Explore AI-powered robotics with NVIDIA's Isaac platform for intelligent robot brains.",
      chapters: {
        'chapter4': {
          title: "Chapter 4: NVIDIA Isaac Platform",
          content: `NVIDIA Isaac provides AI capabilities for robotics:

🤖 AI-POWERED PERCEPTION
• Computer vision algorithms
• Object recognition
• Scene understanding
• Real-time inference

🧭 NAVIGATION & PLANNING
• Path planning algorithms
• Obstacle avoidance
• SLAM (Simultaneous Localization and Mapping)
• Autonomous navigation

🚀 PERFORMANCE FEATURES
• GPU-accelerated processing
• Pre-trained models
• Containerized deployment
• Cloud robotics support

Isaac enables robots to perceive, learn, and adapt to complex environments.`
        }
      },
      keywords: ['nvidia', 'isaac', 'ai', 'gpu', 'perception', 'navigation']
    },
    'module4': {
      title: "Module 4: Vision-Language-Action (VLA)",
      description: "Master multi-modal AI systems that integrate vision, language, and action for advanced robotics.",
      chapters: {
        'chapter5': {
          title: "Chapter 5: Vision-Language-Action Systems",
          content: `VLA systems integrate multiple AI modalities:

👁️ VISION COMPONENT
• Image and video processing
• Object detection
• Scene understanding
• Visual question answering

💬 LANGUAGE COMPONENT
• Natural language processing
• Text understanding
• Dialogue systems
• Command interpretation

🎯 ACTION COMPONENT
• Task planning
• Motion control
• Decision making
• Execution monitoring

VLA enables robots to understand visual scenes, process language commands, and execute appropriate actions.`
        }
      },
      keywords: ['vla', 'vision', 'language', 'action', 'multi-modal', 'ai systems']
    }
  };

  const urduResponses = {
    'switch_english': "Okay, I'll respond in English from now on.",
    'switch_urdu': "Theek hai, main ab Roman Urdu mein jawab doonga.",
    'help': "Main aapki in topics mein help kar sakta hun:\n\n• Module 1: Robotic Nervous System\n• Module 2: Digital Twin (Gazebo & Unity)\n• Module 3: NVIDIA Isaac\n• Module 4: Vision-Language-Action\n\nAap specific chapter ya topic pooch sakte hain!"
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = () => {
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
    
    // Handle language switching
    if (userInput.includes('roman urdu') || userInput.includes('urdu mein')) {
      setUserPreference('urdu');
      setTimeout(() => {
        const botResponse = {
          id: messages.length + 2,
          text: urduResponses['switch_urdu'],
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
          text: urduResponses['switch_english'],
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botResponse]);
        setIsTyping(false);
      }, 1000);
      return;
    }

    // Process query with smart matching
    setTimeout(() => {
      const botResponse = generateSmartResponse(userInput);
      setMessages(prev => [...prev, botResponse]);
      setIsTyping(false);
    }, 1500);
  };

  const generateSmartResponse = (userInput) => {
    // Module 1 queries
    if (userInput.includes('module 1') || userInput.includes('robotic nervous system') || 
        userInput.includes('chapter1') || userInput.includes('chapter 1') ||
        userInput.includes('sensory') || userInput.includes('motor system') || userInput.includes('processing unit')) {
      const content = smartContent.module1.chapters.chapter1;
      return {
        id: messages.length + 2,
        text: `🤖 ${smartContent.module1.title}\n\n${content.content}\n\n💡 This module teaches you how to build robotic systems that can perceive, think, and act like biological organisms.`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // Module 2 - Gazebo
    if (userInput.includes('gazebo') || userInput.includes('chapter2') || userInput.includes('chapter 2') || 
        userInput.includes('simulation') || userInput.includes('3d environment')) {
      const content = smartContent.module2.chapters.chapter2;
      return {
        id: messages.length + 2,
        text: `🖥️ ${smartContent.module2.title}\n\n${content.content}\n\n💡 Gazebo helps you test robots safely before real-world deployment.`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // Module 2 - Unity
    if (userInput.includes('unity') || userInput.includes('chapter3') || userInput.includes('chapter 3') ||
        userInput.includes('digital twin') || userInput.includes('visualization')) {
      const content = smartContent.module2.chapters.chapter3;
      return {
        id: messages.length + 2,
        text: `🎮 ${smartContent.module2.title}\n\n${content.content}\n\n💡 Unity creates realistic digital twins for advanced robotics applications.`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // Module 3 - NVIDIA Isaac
    if (userInput.includes('module 3') || userInput.includes('nvidia') || userInput.includes('isaac') || 
        userInput.includes('chapter4') || userInput.includes('chapter 4') || userInput.includes('ai robot') ||
        userInput.includes('gpu') || userInput.includes('perception')) {
      const content = smartContent.module3.chapters.chapter4;
      return {
        id: messages.length + 2,
        text: `🧠 ${smartContent.module3.title}\n\n${content.content}\n\n💡 NVIDIA Isaac gives robots advanced AI capabilities for complex tasks.`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // Module 4 - VLA
    if (userInput.includes('module 4') || userInput.includes('vla') || userInput.includes('vision language') ||
        userInput.includes('chapter5') || userInput.includes('chapter 5') || userInput.includes('multi-modal') ||
        userInput.includes('vision') && userInput.includes('language') && userInput.includes('action')) {
      const content = smartContent.module4.chapters.chapter5;
      return {
        id: messages.length + 2,
        text: `🌟 ${smartContent.module4.title}\n\n${content.content}\n\n💡 VLA represents the future of human-robot interaction through multi-modal AI.`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // General module queries
    if (userInput.includes('module 2') || userInput.includes('digital twin')) {
      return {
        id: messages.length + 2,
        text: `📚 ${smartContent.module2.title}\n\n${smartContent.module2.description}\n\nAvailable Chapters:\n• Chapter 2: Gazebo Simulation Environment\n• Chapter 3: Unity Integration\n\nWhich chapter would you like to learn about?`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // Help and greetings
    if (userInput.includes('hello') || userInput.includes('hi') || userInput.includes('hey')) {
      if (userPreference === 'urdu') {
        return {
          id: messages.length + 2,
          text: "Salaam! Main aapka Robotics documentation assistant hun. Aap mujhse kisi bhi module ke bare mein pooch sakte hain:\n\n• Module 1: Robotic Nervous System\n• Module 2: Digital Twin\n• Module 3: NVIDIA Isaac\n• Module 4: Vision-Language-Action\n\nAap kya janna chahte hain?",
          sender: 'bot',
          timestamp: new Date(),
        };
      }
      return {
        id: messages.length + 2,
        text: `Hello! I can help you explore these robotics modules:

🤖 Module 1: The Robotic Nervous System
🖥️ Module 2: Digital Twin (Gazebo & Unity)  
🧠 Module 3: AI Robot Brain (NVIDIA Isaac)
🌟 Module 4: Vision-Language-Action (VLA)

What would you like to learn about?`,
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    if (userInput.includes('thank')) {
      if (userPreference === 'urdu') {
        return {
          id: messages.length + 2,
          text: "Aapka shukriya! 😊 Koi aur sawal hai?",
          sender: 'bot',
          timestamp: new Date(),
        };
      }
      return {
        id: messages.length + 2,
        text: "You're welcome! 😊 Is there anything else you'd like to know about robotics?",
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    // Default help response
    if (userPreference === 'urdu') {
      return {
        id: messages.length + 2,
        text: urduResponses['help'],
        sender: 'bot',
        timestamp: new Date(),
      };
    }

    return {
      id: messages.length + 2,
      text: `I can help you learn about robotics and AI! Try asking about:

• "Tell me about Robotic Nervous System"
• "What is Gazebo simulation?"
• "Explain NVIDIA Isaac platform" 
• "What are VLA systems?"
• "Show me Module 2 chapters"

I have detailed information about all your robotics modules!`,
      sender: 'bot',
      timestamp: new Date(),
    };
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

  return (
    <div className={styles.chatbotContainer}>
      <button 
        className={clsx(styles.chatButton, { [styles.hidden]: isOpen })}
        onClick={toggleChat}
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z" fill="currentColor"/>
        </svg>
        <span>Robotics Assistant</span>
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.botInfo}>
              <div className={styles.botAvatar}>🤖</div>
              <div>
                <h3>Robotics Documentation Expert</h3>
                <span className={styles.status}>
                  <span className={styles.statusIndicator}></span>
                  {userPreference === 'english' ? 'Online' : 'Roman Urdu Mode'}
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
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <div className={styles.inputWrapper}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about robotics modules, chapters, or specific topics..."
                rows="1"
                className={styles.textInput}
              />
              <button 
                onClick={handleSendMessage}
                disabled={inputValue.trim() === ''}
                className={styles.sendButton}
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M22 2L11 13M22 2L15 22L11 13M22 2L2 9L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;