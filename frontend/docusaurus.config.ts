import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Humanoid Robot Book',
  tagline: 'Your comprehensive guide to the world of humanoid robotics.',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'your-org', 
  projectName: 'humanoid-robot-book',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/your-org/humanoid-robot-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/your-org/humanoid-robot-book/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robot Book',
      hideOnScroll: false, // Makes the header sticky
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Chapters',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: '📚 The Roadmap',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/Module1-The robotic-nervous-system-ros-2/intro',
            },
            {
              label: 'Module 2: Digital Twins',
              to: '/docs/Module2-the-digital-twins-gazebo-&-unity/intro',
            },
          ],
        },
        {
          title: '🤝 Connect',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Usrafatima',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/in/yusra-fatima-245967366/',
            },
            {
              label: 'About the Author',
              to: '/about',
            },
          ],
        },
        {
          title: '🚀 Resources',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'Community Discussions',
              href: 'https://github.com/Usrafatima',
            },
          ],
        },
      ],
      copyright: `
        <div style="margin-top: 2rem; opacity: 0.6; font-size: 0.9rem;">
          Copyright © 2025 Humanoid Robot Book. Built with Docusaurus & AI-Native Architecture.<br/>
          Empowering the next generation of roboticists.
        </div>
      `,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

