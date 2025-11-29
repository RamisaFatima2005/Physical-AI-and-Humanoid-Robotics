import React from 'react';
import Layout from '@theme/Layout';
import '../css/custom.css';

export default function BonusAuth() {
  return (
    <Layout title="Authentication Guide" description="Learn about authentication methods">
      <div className="bonus-page">
        <div className="bonus-content">
          <h1 className="bonus-title">Authentication Guide</h1>
          
          <div className="bonus-section">
            <h2>Introduction to Authentication</h2>
            <p>
              Authentication is the process of verifying the identity of a user, device, or system. 
              It ensures that only authorized individuals can access protected resources.
            </p>
          </div>
          
          <div className="bonus-section">
            <h2>Common Authentication Methods</h2>
            <ul>
              <li>Username and Password</li>
              <li>Multi-Factor Authentication (MFA)</li>
              <li>Biometric Authentication</li>
              <li>OAuth and OpenID Connect</li>
              <li>JSON Web Tokens (JWT)</li>
            </ul>
          </div>
          
          <div className="bonus-section">
            <h2>Best Practices</h2>
            <p>
              Always use strong passwords, enable MFA where possible, and regularly update 
              your authentication mechanisms to stay secure against emerging threats.
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}