import React, { useEffect, useRef } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';
import '../css/custom.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  const heroRef = useRef(null);
  const featuresRef = useRef(null);

  useEffect(() => {
    // Add scroll animations
    const observerOptions = {
      threshold: 0.1,
      rootMargin: '0px 0px -50px 0px'
    };

    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('animate-in');
        }
      });
    }, observerOptions);

    // Observe feature cards and stats
    document.querySelectorAll('.feature-card, .stat-item, .section-header').forEach(el => {
      observer.observe(el);
    });

    return () => observer.disconnect();
  }, []);

  return (
    <Layout
      title={`${siteConfig.title} - Home`}
      description="Comprehensive guide to modern technologies and development practices">
      
      <main className="main-content">
        {/* Animated Background Elements */}
        <div className="floating-shapes">
          <div className="shape shape-1"></div>
          <div className="shape shape-2"></div>
          <div className="shape shape-3"></div>
          <div className="shape shape-4"></div>
        </div>

        {/* Hero Section */}
        <section className="hero-section" ref={heroRef}>
          <div className="hero-background">
            <div className="hero-particles" id="particles-js"></div>
            <div className="hero-overlay"></div>
          </div>
          <div className="container">
            <div className="hero-content">
              <div className="hero-badge animated-badge">
                <span>🚀 Premium Content</span>
                <div className="badge-sparkle"></div>
              </div>
              
              <h1 className="hero-title gradient-text animated-title">
                {siteConfig.title}
              </h1>
              
              <p className="hero-subtitle animated-subtitle">
                {siteConfig.tagline}
              </p>

              {/* Animated Stats */}
              <div className="hero-stats">
                <div className="stat-item">
                  <div className="stat-icon">📚</div>
                  <span className="stat-number" data-count="3">0</span>
                  <span className="stat-label">Bonus Chapters</span>
                </div>
                <div className="stat-item">
                  <div className="stat-icon">⚡</div>
                  <span className="stat-number" data-count="100">0</span>
                  <span className="stat-label">Practical</span>
                </div>
                <div className="stat-item">
                  <div className="stat-icon">🌐</div>
                  <span className="stat-number" data-count="24">0</span>
                  <span className="stat-label">Access</span>
                </div>
              </div>

              {/* Scroll Indicator */}
              <div className="scroll-indicator">
                <div className="scroll-arrow"></div>
              </div>
            </div>
          </div>
        </section>

        {/* Features Grid */}
        <section className="features-section" ref={featuresRef}>
          <div className="container">
            <div className="section-header">
              <h2>Featured Chapters</h2>
              <p>Explore our comprehensive bonus content</p>
              <div className="section-decoration">
                <div className="decoration-line"></div>
                <div className="decoration-dot"></div>
                <div className="decoration-line"></div>
              </div>
            </div>
            
            <div className="features-grid">
              {/* Security Card */}
              <div className="feature-card security-card">
                <div className="card-glow"></div>
                <div className="card-icon">
                  <div className="icon-wrapper pulse-animation">
                    <span className="icon-shield">🛡️</span>
                  </div>
                </div>
                <div className="card-content">
                  <h3>Authentication & Security</h3>
                  <p>Master modern authentication methods, security protocols, and best practices for secure applications.</p>
                  <ul className="feature-list">
                    <li><span className="list-icon">🔐</span> OAuth 2.0 & OpenID Connect</li>
                    <li><span className="list-icon">🗝️</span> JWT & Session Management</li>
                    <li><span className="list-icon">📱</span> Multi-Factor Authentication</li>
                    <li><span className="list-icon">🛡️</span> Security Best Practices</li>
                  </ul>
                </div>
                <div className="card-actions">
                  <a href="/docs/Module-1-The%20Robotic-Nervous-System/chapter1" className="btn btn-primary glow-on-hover">
                    <span>Explore Chapter</span>
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
                      <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                  </a>
                </div>
              </div>

              {/* Personalization Card */}
              <div className="feature-card personalization-card">
                <div className="card-glow"></div>
                <div className="card-icon">
                  <div className="icon-wrapper rotate-animation">
                    <span className="icon-personalize">🎯</span>
                  </div>
                </div>
                <div className="card-content">
                  <h3>Personalization</h3>
                  <p>Learn to create tailored user experiences with advanced personalization techniques and AI-driven recommendations.</p>
                  <ul className="feature-list">
                    <li><span className="list-icon">📊</span> User Behavior Analysis</li>
                    <li><span className="list-icon">🤖</span> Recommendation Engines</li>
                    <li><span className="list-icon">🎨</span> Adaptive UI/UX</li>
                    <li><span className="list-icon">🧠</span> Machine Learning Integration</li>
                  </ul>
                </div>
                <div className="card-actions">
                  <a href="/docs/Module-1-The%20Robotic-Nervous-System/chapter1" className="btn btn-primary glow-on-hover">
                    <span>Explore Chapter</span>
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
                      <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                  </a>
                </div>
              </div>

              {/* Urdu Card */}
              <div className="feature-card urdu-card">
                <div className="card-glow"></div>
                <div className="card-icon">
                  <div className="icon-wrapper bounce-animation">
                    <span className="icon-urdu">🌍</span>
                  </div>
                </div>
                <div className="card-content urdu-content">
                  <h3 className="urdu-title">اردو گائیڈ</h3>
                  <p className="urdu-text">اردو زبان میں مکمل رہنمائی۔ جدید ٹیکنالوجیز کو اپنی مادری زبان میں سیکھیں۔</p>
                  <ul className="feature-list urdu-text">
                    <li><span className="list-icon">📖</span> مکمل اردو ترجمہ</li>
                    <li><span className="list-icon">🏠</span> مقامی مثالوں کے ساتھ</li>
                    <li><span className="list-icon">💬</span> آسان فہم زبان</li>
                    <li><span className="list-icon">🛠️</span> عملی مشقیں</li>
                  </ul>
                </div>
                <div className="card-actions">
                  <a href="/docs/Module-1-The%20Robotic-Nervous-System/chapter1" className="btn btn-primary glow-on-hover">
                    <span>مزید پڑھیں</span>
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
                      <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                  </a>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className="cta-section">
          <div className="cta-background">
            <div className="cta-pattern"></div>
          </div>
          <div className="container">
            <div className="cta-content">
              <div className="cta-badge">
                <span>🎉 Start Learning Today</span>
              </div>
              <h2>Ready to Master These Technologies?</h2>
              <p>Join thousands of developers who have enhanced their skills with our comprehensive guides.</p>
              <div className="cta-buttons">
                <Link 
                  to="/docs/Module-1-The%20Robotic-Nervous-System/chapter1"
                  className="btn btn-outline pulse-btn"
                >
                  <span>View All Chapters</span>
                  <div className="btn-sparkle"></div>
                </Link>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}