"""Drive the microduck demo conversation while recording robot state.

Runs as a zenoh client via the local router. Produces events.jsonl with:
  {"t": ..., "kind": "joints", "pos": [...14 floats...]}
  {"t": ..., "kind": "odom", "pos": [x,y,z], "quat_xyzw": [x,y,z,w]}
  {"t": ..., "kind": "chat", "role": "human"|"agent"|"tool_call"|"tool", "text": ...}
"""

import json
import threading
import time

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.pollen.microduck import assets_fetch
from dimos.simulation.engines.mujoco_shm import ManipShmReader, shm_key_from_path

OUT = "/Users/tule/.claude/jobs/1c2d08cf/tmp/events.jsonl"
NUM_JOINTS = 14

out_f = open(OUT, "w")
out_lock = threading.Lock()
t0 = time.time()


def emit(kind: str, **kw) -> None:
    with out_lock:
        out_f.write(json.dumps({"t": round(time.time() - t0, 4), "kind": kind, **kw}) + "\n")
        out_f.flush()


# --- transports ---------------------------------------------------------
human_tx = make_transport("/human_input")
agent_tx = make_transport("/agent")
odom_tx = make_transport("/odom", PoseStamped)

final_reply = threading.Event()
last_reply = [""]


def on_agent(msg) -> None:
    if isinstance(msg, AIMessage):
        content = msg.content or ""
        tool_calls = getattr(msg, "tool_calls", None) or []
        if isinstance(content, list):  # some models use content blocks
            content = " ".join(str(c) for c in content)
        for tc in tool_calls:
            emit("chat", role="tool_call", text=f"{tc.get('name')}({json.dumps(tc.get('args', {}))})")
        if content:
            emit("chat", role="agent", text=content)
        if content and not tool_calls:
            last_reply[0] = content
            final_reply.set()
    elif isinstance(msg, ToolMessage):
        emit("chat", role="tool", text=str(msg.content)[:300])
    elif isinstance(msg, HumanMessage):
        pass  # our own prompts, logged at publish time


def on_odom(msg) -> None:
    p = msg.position
    q = msg.orientation
    emit("odom", pos=[p.x, p.y, p.z], quat_xyzw=[q.x, q.y, q.z, q.w])


agent_tx.subscribe(on_agent)
odom_tx.subscribe(on_odom)

# --- SHM joint poller ---------------------------------------------------
key = shm_key_from_path(assets_fetch.robot_mjcf_path())
shm = ManipShmReader(key)
stop = threading.Event()


def poll_joints() -> None:
    while not stop.is_set():
        emit("joints", pos=shm.read_positions(NUM_JOINTS))
        time.sleep(1.0 / 30.0)


threading.Thread(target=poll_joints, daemon=True).start()

# --- scripted conversation ---------------------------------------------
print("mesh settling...", flush=True)
time.sleep(8)  # let zenoh link + capture some idle standing frames


def ask(prompt: str, timeout: float) -> None:
    final_reply.clear()
    emit("chat", role="human", text=prompt)
    human_tx.publish(prompt)
    print(f">> {prompt}", flush=True)
    if final_reply.wait(timeout):
        print(f"<< {last_reply[0][:200]}", flush=True)
    else:
        print(f"!! no final reply within {timeout}s", flush=True)
    time.sleep(3)


ask("Who are you?", 90)
ask("What do you see?", 120)
if not ("red" in last_reply[0].lower() and "blue" in last_reply[0].lower()):
    ask("What colors are they?", 90)
ask("Walk towards the red box.", 300)

time.sleep(5)
stop.set()
time.sleep(0.2)
out_f.close()
print("DONE", flush=True)
