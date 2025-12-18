import importlib.resources
from fastapi import FastAPI, WebSocket
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketDisconnect
import asyncio, json, time

import betfsm
from betfsm.jsonvisitor import JsonVisitor
from betfsm.betfsm import (
    BeTFSMRunner, Sequence, ConcurrentSequence,TimedWait,TimedRepeat,
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard, TickingState
        )
from betfsm.logger import get_logger
from collections import deque
import time

from pathlib import Path

app = FastAPI()

# Serve files from the "static" directory at the URL path "/static"
frontend_path = importlib.resources.files(betfsm) / Path("frontend")
get_logger().warn(f"frontend_path={frontend_path}")
app.mount("/static", StaticFiles(directory=str(frontend_path),html=True), name="static")


clients = set()
root = None   # root state machine to display. (set from outside)

def set_state_machine(sm):
    global root
    root = sm

class HistoryBuffer:
    def __init__(self, seconds=300):
        self.seconds = seconds
        self.buffer = deque()

    def append(self, active_ids):
        now = time.time()
        self.buffer.append((now, tuple(active_ids)))
        cutoff = now - self.seconds
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()

    def slice(self, start_ts, end_ts):
        return [(ts, ids) for ts, ids in self.buffer if start_ts <= ts <= end_ts]

history = HistoryBuffer(seconds=300)



# Capture the main loop once the app starts
main_loop: asyncio.AbstractEventLoop | None = None

@app.on_event("startup")
async def on_startup():
    global main_loop
    main_loop = asyncio.get_running_loop()
    
class Broadcaster:
    def broadcast(self, message: dict):
        if main_loop is None:
            return  # loop not ready yet
        data = json.dumps(message)
        dead = []
        for ws in list(clients):
            # schedule the send on the main loop from this thread
            try:
                asyncio.run_coroutine_threadsafe(ws.send_text(data), main_loop)
            except Exception:
                dead.append(ws)
        # prune dead clients
        for ws in dead:
            clients.discard(ws)
            
            
broadcaster = Broadcaster()

@app.get("/api/alive")
def alive():
    return JSONResponse({"alive":1}) 

@app.get("/api/tree")
def get_tree():
    get_logger().info("/api/tree called")
    global root
    visitor = JsonVisitor()
    if root is not None:
       root.accept(visitor)
    else:
        get_logger().warn("No state machine is set to display (root==None)")
    return JSONResponse(visitor.result())

@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            try:
                await asyncio.sleep(1)
                # Send current active states periodically to keep connection alive
                active_ids = list(TickingState.get_global_log().keys())
                msg = {"type": "tick", "tick": int(time.time()*1000), "active": active_ids}
                await ws.send_text(json.dumps(msg))
            except (RuntimeError, WebSocketDisconnect):
                # Client disconnected, break the loop
                break
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        clients.discard(ws)

@app.get("/api/history")
def get_history(from_ts: float, to_ts: float):
    get_logger().warn("/api/history called")
    frames = history.slice(from_ts, to_ts)
    resp =  JSONResponse([{"timestamp": ts, "active": ids} for ts, ids in frames])
    get_logger().info(f"/api/history response = {resp}")
    return resp

def publish_tick():
    active_ids = list(TickingState.global_publish_log.keys())
    history.append(active_ids)
    msg = {"type": "tick", "tick": int(time.time()*1000), "active": active_ids}
    broadcaster.broadcast(msg)


