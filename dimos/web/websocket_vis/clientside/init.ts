import { io } from "npm:socket.io-client"
import { decode } from "./decoder.ts"
import { Drawable, EncodedSomething } from "./types.ts"
import { Visualizer as ReactVisualizer } from "./vis2.tsx"

// Store server state locally
let serverState = {
    status: "disconnected",
    connected_clients: 0,
    data: {},
    draw: {},
}

let reactVisualizer: ReactVisualizer | null = null

const socket = io()

socket.on("connect", () => {
    console.log("Connected to server")
    serverState.status = "connected"
})

socket.on("disconnect", () => {
    console.log("Disconnected from server")
    serverState.status = "disconnected"
})

socket.on("message", (data) => {
    console.log("Received message:", data)
})

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
            deepMerge(source[key], destination[key])
        } else {
            // Otherwise, just copy the value
            destination[key] = source[key]
        }
    }
    return destination
}

type DrawConfig = { [key: string]: any }

type EncodedDrawable = EncodedSomething
type EncodedDrawables = {
    [key: string]: EncodedDrawable
}
type Drawables = {
    [key: string]: Drawable
}

function decodeDrawables(encoded: EncodedDrawables): Drawables {
    const drawables: Drawables = {}
    for (const [key, value] of Object.entries(encoded)) {
        // @ts-ignore
        drawables[key] = decode(value)
    }
    return drawables
}

function state_update(state: { [key: string]: any }) {
    console.log("Received state update:", state)
    // Use deep merge to update nested properties

    if (state.draw) {
        state.draw = decodeDrawables(state.draw)
    }

    console.log("Decoded state update:", state)
    // Create a fresh copy of the server state to trigger rerenders properly
    serverState = { ...deepMerge(state, { ...serverState }) }

    updateUI()
}

socket.on("state_update", state_update)
socket.on("full_state", state_update)

// Function to send data to server
function emitMessage(data: any) {
    socket.emit("message", data)
}

// Function to update UI based on state
function updateUI() {
    console.log("Current state:", serverState)

    // Update both visualizers if they exist and there's data to display
    if (serverState.draw && Object.keys(serverState.draw).length > 0) {
        if (reactVisualizer) {
            reactVisualizer.visualizeState(serverState.draw)
        }
    }
}

// Initialize the application
function initializeApp() {
    console.log("DOM loaded, initializing UI")
    reactVisualizer = new ReactVisualizer("#vis")

    // Set up click handler to convert clicks to world coordinates and send to server
    reactVisualizer.onWorldClick((worldX, worldY) => {
        emitMessage({ type: "click", position: [worldX, worldY] })
    })

    updateUI()
}

console.log("Socket.IO client initialized")

// Call initialization once when the DOM is loaded
document.addEventListener("DOMContentLoaded", initializeApp)
