// Single source of truth for SimStudio model selection.
// Update these two values only.
export const MODEL_CONFIG = {
  // Used by editor-mode spawned agents AND Scene Builder (vibeCreator)
  editorMode: "gemini-3-flash-preview",
  // Used by sim mode task agent
  simMode: "gemini-robotics-er-1.5-preview",
};

// model: "gemini-3.1-pro-preview",
// model: "gpt-4o",
// model: "gpt-4.1-2025-04-14",          // OpenAI GPT-4.1
// model: "gemini-3-flash-preview",      // Google Gemini Flash
// model: "gemini-robotics-er-1.5-preview",
