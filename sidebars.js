// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  bookSidebar: [
    'why-physical-ai-matters',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: ['chapter1/lesson1', 'chapter1/lesson2', 'chapter1/lesson3'],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: ['chapter2/lesson1', 'chapter2/lesson2', 'chapter2/lesson3'],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: ['chapter3/lesson1', 'chapter3/lesson2', 'chapter3/lesson3'],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: ['chapter4/lesson1', 'chapter4/lesson2', 'chapter4/lesson3'],
    },
  ],
};

export default sidebars;
