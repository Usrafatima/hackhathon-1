import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useLocation } from '@docusaurus/router';
import ChatBot from '../../components/chatbot';

export default function Layout(props) {
  const location = useLocation();
  // Show chatbot only on pages under /docs/
  const isDocsPage = location.pathname.startsWith('/docs');

  return (
    <>
      <OriginalLayout {...props} />
      {isDocsPage && <ChatBot />}
    </>
  );
}
