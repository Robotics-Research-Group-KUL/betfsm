
from fastapi import FastAPI, WebSocket
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
import asyncio, json, time

from backend.jsonvisitor import JsonVisitor
from betfsm.betfsm import (
    BeTFSMRunner, Sequence, ConcurrentSequence,TimedWait,TimedRepeat,
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard, TickingState
        )
from betfsm.logger import get_logger
from collections import deque
import time



app = FastAPI()

# Serve files from the "static" directory at the URL path "/static"
app.mount("/static", StaticFiles(directory="frontend"), name="static")


clients = set()


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

class Broadcaster:
    def broadcast(self, message):
        data = json.dumps(message)
        for ws in list(clients):
            asyncio.create_task(ws.send_text(data))

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

root = {}


@app.get("/api/tree")
def get_tree():
    global root
    visitor = JsonVisitor()
    print(root)
    if root is not None:
       root.accept(visitor)
    return JSONResponse(visitor.result())

@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            await asyncio.sleep(1)
    finally:
        clients.discard(ws)

@app.get("/api/history")
def get_history(from_ts: float, to_ts: float):
    frames = history.slice(from_ts, to_ts)
    return JSONResponse([{"timestamp": ts, "active": ids} for ts, ids in frames])

def publish_tick():
    active_ids = list(TickingState.get_global_log().keys())
    history.append(active_ids)
    msg = {"type": "tick", "tick": int(time.time()*1000), "active": active_ids}
    broadcaster.broadcast(msg)


class BeTFSMRunnerGUI:
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.
    """
    def __init__(self, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, debug: bool = False, display_active=False):
        """
        Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
        runs in the main thread.

        Parameters:
            statemachine:
                the TickingStateMachine to be run
            blackboard:
                the blackboard to be used
            frequency:
                frequency at which the statemachine is ticked (in Hz) (default=100Hz)
            debug:
                If true outputs debug info each tick. (default=False)
        """
        global root
        self.statemachine = statemachine
        self.blackboard = blackboard
        self.frequency = frequency
        self.interval_sec = 1.0/frequency
        self.debug = debug
        self.display_active = display_active
        root = statemachine

    def run(self):
        """
        Runs the statemachine until it returns an outcome different from TICKING

        Refers to absolute time to avoid drifting. Uses monotonic clock to 
        avoid problems with system time changes.

        Returns:
            the final outcome of the statemachine
        """
        # Use monotonic clock to avoid issues with system time changes
        start    = time.monotonic()
        if self.debug:
            get_logger().debug(f"BeTFSMRunner time: {start:10.3f} s started (frequency:{self.frequency})")
        next_run = start + self.interval_sec
        outcome  = TICKING
        now      = start
        while outcome == TICKING:
            outcome = self.statemachine(self.blackboard)
            publish_tick()
            now = time.monotonic()
            if self.display_active:
                gl=TickingState.get_global_log()
                active = "active: ("
                for k,v in TickingState.get_global_log().items():
                    active = active + v.name + " "
                active=active+")"
            else:
                active=""
            if self.debug or self.display_active:
                get_logger().debug(f"{now:10.3f} s : looping {active}")
            # Sleep until the next scheduled time
            sleep_time = next_run - now

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, log drift
                get_logger().warn(f"[Warning] Drift detected: {abs(sleep_time):.3f} s late")

            # Schedule next run
            next_run += self.interval_sec


