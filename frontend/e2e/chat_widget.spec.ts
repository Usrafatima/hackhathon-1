import { test, expect } from '@playwright/test';

test.describe('Chat Widget E2E Tests', () => {
  test('should display chat widget and allow sending messages', async ({ page }) => {
    // Navigate to a Docusaurus page where the chat widget is visible
    await page.goto('http://localhost:3000/docs/intro'); // Assuming Docusaurus is running on 3000

    // Expect chat widget to be visible
    const chatWidgetTitle = page.getByText('Chat with AI');
    await expect(chatWidgetTitle).toBeVisible();

    // Type a message
    const chatInput = page.getByPlaceholderText('Ask a question...');
    await chatInput.fill('Hello from Playwright');

    // Click send button
    await page.getByRole('button', { name: 'Send' }).click();

    // Expect user message to be displayed
    await expect(page.getByText('Hello from Playwright')).toBeVisible();

    // Expect bot response to be displayed (simulated echo)
    await expect(page.getByText(/Echo \(en\): Hello from Playwright/i)).toBeVisible();
  });

  test('should toggle language', async ({ page }) => {
    await page.goto('http://localhost:3000/docs/intro');

    const langButton = page.getByRole('button', { name: 'Lang: EN' });
    await expect(langButton).toBeVisible();

    // Toggle to Urdu
    await langButton.click();
    await expect(page.getByRole('button', { name: 'Lang: UR' })).toBeVisible();

    // Toggle back to English
    await page.getByRole('button', { name: 'Lang: UR' }).click();
    await expect(page.getByRole('button', { name: 'Lang: EN' })).toBeVisible();
  });
});
