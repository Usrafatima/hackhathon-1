import React from 'react';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function Contact(): React.ReactNode {
  return (
    <Layout title="Contact" description="Contact the Humanoid Robot Book team">
      <main className={styles.introSection} style={{ minHeight: '80vh', display: 'flex', alignItems: 'center' }}>
        <div className="container">
          <h1 className={styles.sectionTitle}>Get in Touch</h1>
          <p className={styles.introText}>
            Have questions about the book, or want to collaborate on a humanoid project? 
            We'd love to hear from you.
          </p>
          
          <div style={{ 
            display: 'grid', 
            gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))', 
            gap: '3rem', 
            marginTop: '5rem',
            textAlign: 'left' 
          }}>
            <div className={styles.moduleCard}>
              <h3 style={{ color: '#00F5A0' }}>📧 Email</h3>
              <p>For general inquiries and support:</p>
              <a href="mailto:hello@humanoidrobotbook.com" style={{ color: '#00D4FF', fontWeight: 'bold', fontSize: '1.2rem' }}>
                hello@humanoidrobotbook.com
              </a>
            </div>
            
            <div className={styles.moduleCard}>
              <h3 style={{ color: '#00F5A0' }}>🤝 Community</h3>
              <p>Join the discussion on our GitHub or LinkedIn:</p>
              <div style={{ display: 'flex', gap: '1.5rem', marginTop: '1rem' }}>
                <a href="https://github.com/Usrafatima" target="_blank" rel="noopener noreferrer" style={{ color: '#00D4FF', fontWeight: 'bold' }}>
                  GitHub
                </a>
                <a href="https://www.linkedin.com/in/yusra-fatima-245967366/" target="_blank" rel="noopener noreferrer" style={{ color: '#00D4FF', fontWeight: 'bold' }}>
                  LinkedIn
                </a>
              </div>
            </div>
          </div>

          <div style={{ marginTop: '6rem', opacity: 0.6 }}>
            <p>We typically respond within 24-48 hours during weekdays.</p>
          </div>
        </div>
      </main>
    </Layout>
  );
}
