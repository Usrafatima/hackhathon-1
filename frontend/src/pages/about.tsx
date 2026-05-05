import React from 'react';
import Layout from '@theme/Layout';
import styles from './index.module.css';

export default function About(): React.ReactNode {
  return (
    <Layout title="About" description="About the Humanoid Robot Book">
      <main className={styles.introSection} style={{ minHeight: '80vh', display: 'flex', alignItems: 'center' }}>
        <div className="container">
          <h1 className={styles.sectionTitle}>About the Book</h1>
          <div className={styles.introText} style={{ textAlign: 'left', marginTop: '3rem' }}>
            <p>
              The <strong>Humanoid Robot Book</strong> is a comprehensive roadmap designed for the next generation of engineers, 
              researchers, and hobbyists. As we stand on the brink of a new era in robotics, where humanoid machines 
              move from research labs into our daily lives, there is an urgent need for a unified guide that covers 
              the full spectrum of humanoid engineering.
            </p>
            <p>
              This book is structured to provide a hands-on, project-based learning experience. We believe that 
              true understanding comes from building. That's why every theoretical concept is paired with a 
              practical lab, simulation, or implementation challenge.
            </p>
            <h2 style={{ color: '#00D4FF', marginTop: '3rem' }}>Our Mission</h2>
            <p>
              Our mission is to democratize humanoid robotics education. By leveraging open-source tools like 
              ROS 2, Gazebo, and modern AI frameworks, we aim to make complex robotic systems accessible 
              to anyone with a passion for innovation.
            </p>
            <div style={{ 
              marginTop: '4rem', 
              padding: '2rem', 
              background: 'rgba(0, 245, 160, 0.05)', 
              borderRadius: '20px',
              border: '1px solid rgba(0, 245, 160, 0.1)'
            }}>
              <h3 style={{ color: '#00F5A0' }}>What Sets This Book Apart?</h3>
              <ul>
                <li><strong>AI-Native Approach:</strong> We don't just treat AI as an add-on; it's the core of the robot's brain.</li>
                <li><strong>Digital Twin First:</strong> Master simulation before touching expensive hardware.</li>
                <li><strong>Industry-Standard Tools:</strong> Learn the tools used by companies like Tesla, Boston Dynamics, and Figure.</li>
              </ul>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
