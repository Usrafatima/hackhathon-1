import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import OpenAI from "openai";
import { loadBookDocs } from "./bookloader.js";

let agentClient;
// Commented out OpenAI client initialization to run in browser without API key exposure
// if (ExecutionEnvironment.canUseDOM) {
//     agentClient = new OpenAI({
//       apiKey: "YOUR_OPENAI_API_KEY", // Replace with your actual key
//     });
// }

export { agentClient };
export const bookDocs = loadBookDocs();
