import socketio
import uvicorn
import threading
import os
import sys
import asyncio
from starlette.routing import Route
from starlette.responses import HTMLResponse
from starlette.applications import Starlette
from starlette.staticfiles import StaticFiles


async def serve_index(request):
    # Read the index.html file directly
    index_path = os.path.join(os.path.dirname(__file__), "static", "index.html")
    with open(index_path, "r") as f:
        content = f.read()
    return HTMLResponse(content)


# Create global socketio server
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

# Create Starlette app with route for root path
routes = [Route("/", serve_index)]
starlette_app = Starlette(routes=routes)


static_dir = os.path.join(os.path.dirname(__file__), "static")
starlette_app.mount("/", StaticFiles(directory=static_dir), name="static")

# Create the ASGI app
app = socketio.ASGIApp(sio, starlette_app)

main_state = {"status": "idle", "connected_clients": 0, "data": {}}


@sio.event
async def connect(sid, environ):
    print(f"Client connected: {sid}")
    await update_state({"connected_clients": main_state["connected_clients"] + 1})
    await sio.emit("full_state", main_state, room=sid)


@sio.event
async def disconnect(sid):
    print(f"Client disconnected: {sid}")
    await update_state({"connected_clients": main_state["connected_clients"] - 1})


@sio.event
async def message(sid, data):
    print(f"Message received from {sid}: {data}")
    await sio.emit("message", {"response": "Server received your message"}, room=sid)


# Deep merge function for nested dictionaries
def deep_merge(source, destination):
    """
    Deep merge two dictionaries recursively.
    Updates destination in-place with values from source.
    Lists are replaced, not merged.
    """
    for key, value in source.items():
        if key in destination and isinstance(destination[key], dict) and isinstance(value, dict):
            # If both values are dictionaries, recursively deep merge them
            deep_merge(value, destination[key])
        else:
            # Otherwise, just update the value
            destination[key] = value
    return destination


# Utility function to update state and broadcast to all clients
async def update_state(new_data):
    """Update main_state and broadcast only the new data to all connected clients"""
    # Deep merge the new data into main_state
    deep_merge(new_data, main_state)
    # Broadcast only the new data to all connected clients
    await sio.emit("state_update", new_data)


class WebsocketVis:
    def __init__(self, port=7778, use_reload=False):
        self.port = port
        self.server = None
        self.server_thread = None
        self.sio = sio  # Use the global sio instance
        self.use_reload = use_reload
        self.main_state = main_state  # Reference to global main_state

    def start(self):
        # If reload is requested, run in main thread
        if self.use_reload:
            print("Starting server with hot reload in main thread")
            uvicorn.run(
                "server:app",  # Use import string for reload to work
                host="0.0.0.0",
                port=self.port,
                reload=True,
                reload_dirs=[os.path.dirname(__file__)],
            )
            return self

        # Otherwise, run in background thread
        else:
            print("Starting server in background thread")
            self.server_thread = threading.Thread(
                target=uvicorn.run,
                kwargs={
                    "app": app,  # Use direct app object for thread mode
                    "host": "0.0.0.0",
                    "port": self.port,
                },
                daemon=True,
            )
            self.server_thread.start()
            return self

    def stop(self):
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join()
        self.sio.disconnect()

    async def update_state_async(self, new_data):
        """Update main_state and broadcast to all connected clients"""
        await update_state(new_data)

    def update_state(self, new_data):
        """Synchronous wrapper for update_state"""

        # Get or create an event loop
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        # Run the coroutine in the loop
        if loop.is_running():
            # Create a future and run it in the existing loop
            future = asyncio.run_coroutine_threadsafe(update_state(new_data), loop)
            return future.result()
        else:
            # Run the coroutine in a new loop
            return loop.run_until_complete(update_state(new_data))


# Test timer function that updates state with current Unix time
async def start_time_counter(server):
    """Start a background task that updates state with current Unix time every second"""
    import time

    while True:
        # Update state with current Unix timestamp
        await server.update_state_async({"time": int(time.time())})
        # Wait for 1 second
        await asyncio.sleep(1)


# For direct execution with uvicorn CLI
if __name__ == "__main__":
    # Check if --reload flag is passed
    use_reload = "--reload" in sys.argv
    server = WebsocketVis(port=7778, use_reload=use_reload)
    server_instance = server.start()
