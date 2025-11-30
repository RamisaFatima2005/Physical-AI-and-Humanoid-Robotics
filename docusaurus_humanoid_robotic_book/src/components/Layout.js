import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '@site/src/components/Chatbot'; // ✅ Yahan import karein

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <Chatbot /> {/* ✅ Yahan add karein */}
    </>
  );
}