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
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Explore the Book
          </Link>
        </div>
      </div>
    </header>
  );
}

const features = [
  {
    title: 'AI-Native Learning',
    description: 'Learn to build AI-native software with modern agents and real-world applications.',
    icon: '🤖',
  },
  {
    title: 'Spec-Driven Development',
    description: 'Build software from specifications with clarity, structure, and reproducibility.',
    icon: '📝',
  },
  {
    title: 'Beginner-Friendly + Urdu Support',
    description: 'Includes Urdu explanations, local examples, and simple breakdowns for every learner.',
    icon: '❤️',
  },
];

function FeaturesSection() {
  return (
    <section className="features-section" style={{padding: '4rem 0', backgroundColor: '#0F0F0F', color: '#E8FFF5'}}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div className="col col--4" key={idx} style={{marginBottom: '2rem'}}>
              <div 
                className="feature-card" 
                style={{
                  backgroundColor: '#0A0A0A',
                  borderRadius: '25px',
                  padding: '2rem',
                  textAlign: 'center',
                  color: '#E8FFF5',
                  border: '1px solid #00F5A0',
                  transition: 'transform 0.3s ease-in-out, box-shadow 0.3s ease-in-out',
                }}
                onMouseOver={(e) => {
                  e.currentTarget.style.transform = 'scale(1.05)';
                  e.currentTarget.style.boxShadow = '0 0 20px #00F5A0';
                }}
                onMouseOut={(e) => {
                  e.currentTarget.style.transform = 'scale(1)';
                  e.currentTarget.style.boxShadow = 'none';
                }}
              >
                <div className="feature-icon" style={{fontSize: '3rem', color: '#00D4FF', marginBottom: '1rem'}}>{feature.icon}</div>
                <h3 style={{fontFamily: 'Orbitron, sans-serif', color: '#00F5A0'}}>{feature.title}</h3>
                <p style={{fontFamily: 'Roboto, sans-serif'}}>{feature.description}</p>
              </div>
            </div>
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
      description="Your comprehensive guide to the world of humanoid robotics.">
      
      <HomepageHeader />
      
      <main>
        <FeaturesSection />
      </main>

     
      
    </Layout>
  );
}
