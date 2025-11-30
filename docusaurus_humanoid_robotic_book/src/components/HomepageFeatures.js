import React, { useEffect } from 'react';
import Chatbot from '@site/src/components/Chatbot';

export default function HomepageFeatures() {
  useEffect(() => {
    // Chatbot ko manually DOM mein add karein
    const chatbotContainer = document.createElement('div');
    chatbotContainer.className = 'chatbot-container';
    document.body.appendChild(chatbotContainer);
    
    // React component ko render karein
    import('react-dom').then((ReactDOM) => {
      ReactDOM.default.render(<Chatbot />, chatbotContainer);
    });

    return () => {
      // Cleanup
      document.body.removeChild(chatbotContainer);
    };
  }, []);

  return (
    // Aapka existing homepage content
    <section>
      {/* Your features content */}
    </section>
  );
}