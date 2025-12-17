import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export async function loadBookDocs() {
  if (!ExecutionEnvironment.canUseDOM) {
    return []; // Return empty array during SSR
  }

  const fileNames = [
    'chapter-01.md',
    'chapter-02.md',
    'chapter-03.md',
    'chapter-04.md',
    'chapter-05.md',
    'chapter-06.md',
    'chapter-07.md',
    'chapter-08.md',
    'chapter-09.md',
    'chapter-10.md',
    'chapter-11.md',
    'chapter-12.md',
    'chapter-13.md',
    'chapter-14.md',
    'chapter-15.md',
    'chapter-16.md',
    'intro.md',
  ];

  const bookDocs = await Promise.all(
    fileNames.map(async (fileName) => {
      const response = await fetch(`/docs/${fileName}`);
      const content = await response.text();
      return {
        title: fileName.replace(/\.md$/, ''),
        content: content,
      };
    })
  );

  return bookDocs;
}
