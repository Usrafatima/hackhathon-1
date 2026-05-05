import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <Heading as="h1" className={styles.heroTitle}>
          {siteConfig.title}
        </Heading>
        <p className={styles.heroSubtitle}>
          The definitive engineering roadmap for mastering the Robotic Nervous System, 
          Digital Twins, and Multimodal AI Brains.
        </p>
        <div className={styles.buttons}>
          <Link
            className={styles.primaryButton}
            to="/docs/intro">
            Start Reading
          </Link>
          <Link
            className={clsx('button button--outline button--lg', styles.secondaryButton)}
            style={{ 
              borderRadius: '50px', 
              padding: '1.2rem 2.5rem', 
              color: '#00D4FF', 
              borderColor: '#00D4FF',
              fontWeight: '700'
            }}
            to="/about">
            Learn More
          </Link>
        </div>
      </div>
    </header>
  );
}

function IntroductionSection() {
  return (
    <section className={styles.introSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Engineering Humanoid Intelligence</h2>
        <p className={styles.introText}>
          In an era where AI is moving from digital screens to physical bodies, the 
          Humanoid Robot Book provides the blueprints. This is not just a textbook; 
          it's an interactive journey through the full stack of modern robotics—from 
          real-time hardware communication and high-fidelity physics simulations to 
          generative AI decision-making.
        </p>
      </div>
    </section>
  );
}

const modules = [
  {
    number: 'Module 01',
    title: 'The Robotic Nervous System',
    description: 'Master ROS 2 architecture, hardware abstraction layers, and low-latency communication protocols essential for real-time humanoid control.',
  },
  {
    number: 'Module 02',
    title: 'Digital Twins & Simulation',
    description: 'Bridge the reality gap with Gazebo and Unity. Learn to build high-fidelity digital twins for safe, iterative testing of complex robot behaviors.',
  },
  {
    number: 'Module 03',
    title: 'Cognitive AI Brain',
    description: 'Implement advanced neural controllers, decision-making agents, and reinforcement learning strategies that drive autonomous humanoid action.',
  },
  {
    number: 'Module 04',
    title: 'Perception & Interaction',
    description: 'Equip robots with the ability to see and speak. Explore computer vision, SLAM, and integration with multimodal LLMs for natural interaction.',
  },
];

function CurriculumSection() {
  return (
    <section className={styles.curriculumSection}>
      <div className="container">
        <h2 className={styles.sectionTitle} style={{textAlign: 'center'}}>Curriculum Overview</h2>
        <div className={styles.moduleGrid}>
          {modules.map((module, idx) => (
            <div key={idx} className={styles.moduleCard}>
              <div className={styles.moduleNumber}>{module.number}</div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <p className={styles.moduleDescription}>{module.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsSection() {
  const stats = [
    { number: '4', label: 'Core Modules' },
    { number: '20+', label: 'Practical Labs' },
    { number: '100%', label: 'Open Source' },
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          {stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <span className={styles.statNumber}>{stat.number}</span>
              <span className={styles.statLabel}>{stat.label}</span>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function AudienceSection() {
  const audiences = [
    { icon: '🎓', label: 'Students' },
    { icon: '🏗️', label: 'Roboticists' },
    { icon: '🧠', label: 'AI Researchers' },
  ];

  return (
    <section className={styles.audienceSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Target Audience</h2>
        <div className={styles.audienceGrid}>
          {audiences.map((audience, idx) => (
            <div key={idx} className={styles.audienceItem}>
              <div className={styles.audienceIcon}>{audience.icon}</div>
              <div className={styles.audienceLabel}>{audience.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function TechStackSection() {
  const techs = ['ROS 2', 'Gazebo', 'Unity', 'Python', 'C++', 'PyTorch', 'Cohere', 'Qdrant'];
  return (
    <section className={styles.statsSection} style={{ background: '#0A0A0A', borderBottom: '1px solid rgba(0, 245, 160, 0.1)' }}>
      <div className="container">
        <h3 style={{ 
          fontFamily: 'Orbitron', 
          textAlign: 'center', 
          color: '#00D4FF', 
          marginBottom: '3rem',
          fontSize: '1rem',
          letterSpacing: '5px',
          opacity: 0.6
        }}>CORE TECHNOLOGIES</h3>
        <div style={{ 
          display: 'flex', 
          justifyContent: 'center', 
          flexWrap: 'wrap', 
          gap: '3rem',
          alignItems: 'center',
          opacity: 0.8
        }}>
          {techs.map((tech, idx) => (
            <span key={idx} style={{ 
              fontFamily: 'Orbitron', 
              fontSize: '1.2rem', 
              color: '#E8FFF5',
              fontWeight: 700
            }}>{tech}</span>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="The definitive guide to the future of humanoid robotics and AI.">
      <main>
        <HomepageHeader />
        <StatsSection />
        <IntroductionSection />
        <CurriculumSection />
        <TechStackSection />
        <AudienceSection />
      </main>
    </Layout>
  );
}
