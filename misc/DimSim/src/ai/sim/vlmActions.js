import { MODEL_CONFIG } from "../modelConfig.js";

// Sim-only action surface for SimStudio/DimSim parity.
// Keep this list free of editor/build actions.

export const ACTIONS = [
  // === MOVEMENT ===
  {
    id: "MOVE_FORWARD",
    description: "Move forward. Use for approaching things you see.",
    params: { steps: "integer 1-5" },
  },
  {
    id: "MOVE_BACKWARD",
    description: "Move backward. Use to back away or reposition.",
    params: { steps: "integer 1-3" },
  },
  {
    id: "STRAFE_LEFT",
    description: "Sidestep left without turning.",
    params: { steps: "integer 1-3" },
  },
  {
    id: "STRAFE_RIGHT",
    description: "Sidestep right without turning.",
    params: { steps: "integer 1-3" },
  },
  {
    id: "MOVE_UP",
    description: "Move upward (float up) for vertical repositioning in sim.",
    params: { steps: "integer 1-3" },
  },
  {
    id: "MOVE_DOWN",
    description: "Move downward (float down) for vertical repositioning in sim.",
    params: { steps: "integer 1-3" },
  },

  // === LOOKING/TURNING ===
  {
    id: "TURN_LEFT",
    description: "Turn body left (yaw). Use to see what's to your left or explore new directions.",
    params: { degrees: "number 30-90" },
  },
  {
    id: "TURN_RIGHT",
    description: "Turn body right (yaw). Use to see what's to your right or explore new directions.",
    params: { degrees: "number 30-90" },
  },
  {
    id: "LOOK_UP",
    description: "Tilt view upward. Use to see shelves, ceilings, tall objects.",
    params: { degrees: "number 15-45" },
  },
  {
    id: "LOOK_DOWN",
    description: "Tilt view downward. Use to see floor, low objects, items on ground.",
    params: { degrees: "number 15-45" },
  },

  // === NAVIGATION ===
  {
    id: "GOTO_LOCATION",
    description: "Navigate toward a known location. Use 'start' to return to where you began.",
    params: { locationId: "string (tag id from nearbyLocations, or 'start')" },
  },

  // === INTERACTION ===
  {
    id: "INTERACT",
    description: "Interact with an object. REQUIRES: object in NEARBY OBJECTS list AND distance < 1.5m AND object should be in your field of vision. Use the EXACT assetId from [id: ...] brackets!",
    params: { assetId: "string (EXACT id from [id: xxx] in NEARBY OBJECTS)", actionLabel: "string (from can: list)" },
  },

  // === PICK UP / DROP ===
  {
    id: "PICK_UP",
    description: "Pick up a pickable object. REQUIRES: object marked [pickable] in NEARBY OBJECTS AND distance < 1.5m AND you're not already holding something AND object should be in your field of vision.",
    params: { assetId: "string (EXACT id from [id: xxx] in NEARBY OBJECTS)" },
  },
  {
    id: "DROP",
    description: "Drop the object you're currently holding. Places it in front of you.",
    params: {},
  },

  // === META ===
  {
    id: "THINK",
    description: "Pause to reason about your situation. Use when stuck or need to reconsider your approach.",
    params: { thought: "string (your reasoning)" },
  },
  {
    id: "DONE",
    description: "Task is complete. Only use when you've achieved the goal.",
    params: { summary: "string (what you accomplished)" },
  },
];

export const DEFAULTS = {
  model: MODEL_CONFIG.simMode,
  decideEverySteps: 6,
  stepMeters: 0.4,
  maxToiMeters: 50,
};
// model: "gpt-4o",
// model: "gemini-3.1-pro-preview",
// model: "gemini-3-flash-preview",      // Google Gemini Flash
// model: "gemini-robotics-er-1.5-preview",
