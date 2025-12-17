import BookChatWidget from "../components/chatbot";
import React, { useState, useEffect } from 'react';

function Root({ children }) {
  const [selectedText, setSelectedText] = useState<string>('');

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
       <BookChatWidget/>
      {children}
   
    </>
  );
}

export default Root;