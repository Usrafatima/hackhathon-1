import BookChatWidget from "../components/chatbot";
import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

function Root({ children }) {
  const [selectedText, setSelectedText] = useState<string>('');
  const location = useLocation();
  const showChatbot = location.pathname.startsWith('/docs');

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        setSelectedText(selection.toString());
      } else {
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  return (
    <>
      {showChatbot && <BookChatWidget />}
      {children}
    </>
  );
}

export default Root;