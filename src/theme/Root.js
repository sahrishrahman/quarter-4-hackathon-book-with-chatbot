import React from 'react';
import ChatbotSidebar from '@site/src/components/ChatbotSidebar';

// Default implementation, that you can customize
function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotSidebar />
    </>
  );
}

export default Root;