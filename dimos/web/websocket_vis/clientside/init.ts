import { io } from "npm:socket.io-client";
import pickleparser from "npm:pickleparser";

const parser = new pickleparser.Parser();

// Store server state locally
let serverState = {
  status: "disconnected",
  connected_clients: 0,
  data: {},
};

const socket = io();

socket.on("connect", () => {
  console.log("Connected to server");
  serverState.status = "connected";
});

socket.on("disconnect", () => {
  console.log("Disconnected from server");
  serverState.status = "disconnected";
});

socket.on("message", (data) => {
  console.log("Received message:", data);
});

// Deep merge function for client-side state updates
function deepMerge(source: any, destination: any): any {
  for (const key in source) {
    // If both source and destination have the property and both are objects, merge them
    if (
      key in destination &&
      typeof source[key] === "object" &&
      source[key] !== null &&
      typeof destination[key] === "object" &&
      destination[key] !== null &&
      !Array.isArray(source[key]) &&
      !Array.isArray(destination[key])
    ) {
      deepMerge(source[key], destination[key]);
    } else {
      // Otherwise, just copy the value
      destination[key] = source[key];
    }
  }
  return destination;
}

type DrawConfig = { [key: string]: any };
type Drawable = { config: DrawConfig; data: ArrayBuffer };
type StateUpdate = { [entity: string]: Drawable };

function update_entity(name: string, data?: ArrayBuffer, config?: DrawConfig) {
  // ensure that data exists and is ArrayBuffer
  if (!data || !(data instanceof ArrayBuffer)) {
    return;
  }

  console.log("update_entity", name, data, config);
  console.log(parser.parse(new Uint8Array(data)));
}

function state_update(state: StateUpdate) {
  console.log("Received partial state update:", state);
  // Use deep merge to update nested properties
  serverState = deepMerge(state, { ...serverState });

  for (const [key, value] of Object.entries(state)) {
    // Handle the entity update
    update_entity(key, value.data, value.config);
  }

  // Trigger UI updates
  updateUI();
}

socket.on("state_update", state_update);
socket.on("full_state", state_update);

// Function to send data to server
function sendData(data: any) {
  socket.emit("message", data);
}

// Function to update UI based on state
function updateUI() {
  console.log("Current state:", serverState);

  // Find the element with id="json"
  const jsonElement = document.getElementById("json");

  // If the element exists, update its content with the formatted JSON
  if (jsonElement) {
    // Pretty print the JSON with 2 space indentation
    jsonElement.textContent = JSON.stringify(serverState, null, 2);
  } else {
    console.warn("Element with id='json' not found in the DOM");
  }
}

console.log("Socket.IO client initialized");

// Call updateUI once when the DOM is loaded
document.addEventListener("DOMContentLoaded", () => {
  console.log("DOM loaded, initializing UI");
  updateUI();
});
