# app.py - backend server for visualization and debugging using HTTP
#
#region Copyright (C) Erwin Aertbeliën, 2025
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
#endregion

from typing import Any


import importlib.resources
from fastapi import FastAPI, Response, WebSocket, HTTPException
from fastapi.responses import JSONResponse, FileResponse,HTMLResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketDisconnect
import asyncio, json, time
from fastapi.middleware.cors import CORSMiddleware

import betfsm
from betfsm.betfsm import TickingState,json_serializer
from betfsm.events import HTTPEventReceiver
from betfsm.jsonvisitor import to_json
from betfsm.logger import get_logger
from collections import deque
import time

from pathlib import Path
from pydantic import BaseModel, Field

#region FastAPI description
description="""
This is the **API** documentationfor **BeTFSM**

BeTFSM is a library for "ticking" statemachines and behavior trees. In this
unified framework, discrete tasks can be specified with the modularity of
behavior trees and at the lower-level precise interaction can be specified
using state machines. It targets discrete coordination of robotic systems at
both high- and low level.


It consists of multiple packages:

  - **betfsm** contains the core library, only dependent on python packages, e.g. the implementation of a set of state machines and behavior tree nodes, such as [Sequence](sequence.md), [Fallback](fallback.md), [Repeat](repeat.md), [ConcurrentSequence](concurrentsequence.md), ...
  - **betfsm_ros** contains facilities to use BeTFSM on [ROS2](https://github.com/ros2), e.g. managing timing using ROS2, calling services, calling actions, ...
  - **betfsm_crospi** contains facilities to use BeTFSM with [cROSpi](https://github.com/Robotics-Research-Group-KUL/crospi), e.g. communicating with cROSpi over ROS2, managing the lifecycle of a task, sending over parameters to a cROSpi task, ...

Documentation of BeTFSM can be found [here](https://robotics-research-group-kul.github.io/betfsm/).

Published under the GNU LESSER GENERAL PUBLIC LICENSE Version 3, 29 June 2007.

(c) 2024-2026, Erwin Aertbeliën

 KU Leuven, Department of Mechanical Engineering, ROB-Group. The ROB Group is part of core labs M&A and MPRO of Flanders Make.


## WebSocket: /ws/stream

**URL:** `ws://yourserver/ws/stream

**Protocol:** WebSocket  
**Description:** The nodes in the tree that have to be painted as active.

The message is in json:

    ```
    msg = {"type": "tick", "tick": time_in_milliseconds, "active": list of active UUID's of nodes}
    ```


"""
app = FastAPI(
    title="BeTFSM webserver API",
    description=description,
    license_info={
        "name": "GNU LESSER GENERAL PUBLIC LICENSE Version 3, 29 June 2007",
        "url": "https://www.gnu.org/licenses/lgpl-3.0.en.html",
    },    
)
#endregion

# To deal with browser CORS protocol and being able to send
# from all origins:
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],        # allow all origins (or restrict later)
    allow_credentials=True,
    allow_methods=["*"],        # allow POST, GET, OPTIONS, etc.
    allow_headers=["*"],        # allow Content-Type, Accept, etc.
)



#region Models for the API:

class Event(BaseModel):
    channel: str = Field("betfsm", 
                         description="Name of the channel, i.e. a BeTFSM receiver subscribes to channel and will have a queue of all events for that channel.",
                         max_length=30)
    event:   str = Field("", 
                         description="Name of the event",
                         max_length=30)

class SetValueRequest(BaseModel):
    value: Any = Field(
        ...,
        description="The value to store at the given path. Can be any valid JSON type."
    )
#endregion

#region GLOBAL VARIABLES   (not valid if multiple workers)
clients = set()
root = None   # root state machine to display. (set from outside)
blackboard = None
name_filter = None
type_filter = None
branding    = True
tree_version = 1
frontend_path = importlib.resources.files(betfsm) / Path("frontend") # Serve files from the "static" directory at the URL path "/static"
# get_logger().info(f"frontend_path={frontend_path}")
#endregion


app.mount("/static", StaticFiles(directory=str(frontend_path),html=True), name="static")


def set_webserver_param(sm:TickingState, bb:dict,my_name_filter:str|None, my_type_filter:str|None, my_branding):
    """
    Sets parameters for the webserver:

    Parameters:
        sm:
            A BeTFSM tree
        bb:
            Blackboard (hierarchical tree using Python's dict)
        my_name_filter:
            do not descend further into nodes that match the names in this list.
            The name can be a regular expression.
        my_type_filter:
            do not descend further into nodes that are an instance of the types in this list.
    """
    global root, name_filter, type_filter,blackboard,branding
    root = sm
    blackboard = bb
    name_filter = my_name_filter
    type_filter = my_type_filter
    branding    = my_branding


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


@app.get("/",summary="Get main page of visualisation GUI")
def main_page():
    global branding
    if branding:
        # Point to your frontend index.html inside the betfsm/frontend directory
        index_file = frontend_path / "index.html"
    else:
        index_file = frontend_path / "index_nb.html"
    return FileResponse(str(index_file))


@app.get("/dashboard",summary="Get GUI panel to examine the blackboard")
async def get_dashboard():
    index_file = frontend_path / "dashboard.html"
    return FileResponse(str(index_file))


@app.get("/events",summary="get GUI panel to send out HTTP events")
async def get_events():
    index_file = frontend_path / "events.html"
    return FileResponse(str(index_file))


@app.get("/favicon.ico")
async def get_dashboard():
    index_file = frontend_path / "favicon.ico"
    return FileResponse(str(index_file))


@app.post("/api/set_root_name",
         summary="sets the root of the tree that /api/get_tree returns"
         )
async def set_root_name(root_name:str):
    global name_filter,tree_version
    name_filter = root_name    
    tree_version += 1
    get_logger().info(f"root_name set to '{root_name}'")
    return {"message": "Value updated", "current_value": name_filter}


@app.get("/api/get_root_name",
         summary="gets the root of the tree that /api/get_tree returns"
         )
async def get_root_name():
    global name_filter
    return {"root_name":name_filter}


@app.post("/api/set_type_filter",
         summary="defines which types not to descend into when /api/get_tree is called, the filter is specified using a ':' separated list of types"
         )
async def set_type_filter(filter:str):
    global type_filter,tree_version
    type_filter = filter 
    tree_version += 1
    get_logger().info(f"type_filter set to '{filter}'")
    return {"message": "Value updated", "current_value": type_filter}


@app.get("/api/get_type_filter",
         summary="gets the ':'-separated list of types that determins which nodes not to descend into when /api/get_tree is called."
         )
async def get_type_filter():
    global type_filter
    return type_filter


@app.get('/api/event_list',
         summary="returns lists of events used by BeTFSM",
         description="""
A dictionary that maps a type of condition list of events used *somewhere* by that condition.  This is a static 
description, generated during construction.  This are lists of events **accessible** by the condition, not only
the events that this type of condition actually generates.
""",
response_description="a dictionary that maps BeTFSM Condition Classes to a list of events.")
async def send_event_list():
    get_logger().info("Received request for a list of events used in BeTFSM")
    return betfsm.Condition.registry

@app.post(
    "/api/event",
    summary="Push an event towards a HTTPEventReceiver",
    response_description=""
)
async def receive_event(event: Event):
    get_logger().info(f"HTTP event received: channel: {event.channel} event:{event.event}")
    instance = HTTPEventReceiver.lookup_instance(event.channel)
    if instance is not None:
        instance.push(event.event)
    return {"status":"ok"}


@app.get(
    "/api/blackboard/{path:path}",
    summary="Get a value from the blackboard",
    description= 
        """
Returns the value stored at the given hierarchical path.  

- Each path component is separated with '/'. 
- List elements are adressed by numeric indices, e.g. /output/0/name
- length is obtained using '~len', e.g. /output/~len
        """
)
async def get_blackboard_value(path: str):
    get_logger().info(f"HTTP REQUEST : get_blackboard_value {path}")
    value = betfsm.get_path_value(blackboard, path)
    if value is None:
        raise HTTPException(status_code=404, detail="Path not found")
    result = {"path": path, "value": value}
    json_string = json.dumps(result, default=json_serializer)
    return Response(content=json_string, media_type="application/json")




@app.put(
    "/api/blackboard/{path:path}",
    summary="Set a value in the blackboard",
    description=
"""
Stores a value at the given hierarchical path, creating intermediate nodes if needed.
A path and a value should be given.
The path has the following syntax:

- '~append' appends a new value to a list
- '~insert:N' inserts a value at index N, e.g. /output/~insert:2
- "~del:N" deletes a value at index N
- "~pop~ removes the last element
- "N" replaces the element at index N
"""
)
async def set_blackboard_value(path: str, req: SetValueRequest):
    get_logger().info(f"HTTP REQUEST : set_blackboard_value {path}")
    betfsm.set_path_value(blackboard, path, req.value)
    return {"path": path, "value": req.value}


@app.get("/api/alive", summary="Ping server to see if it is alive")
def alive():
    global tree_version
    return JSONResponse({"alive":1,"tree_version":tree_version}) 


@app.get("/api/tree", summary="Get the BeTFSM tree in JSON")
def get_tree():
    global root
    get_logger().info("/api/tree called")
    jsondict = to_json(root,type_filter,name_filter)
    return JSONResponse(jsondict)


@app.websocket("/ws/stream")
async def ws_stream(ws: WebSocket):
    global tree_version
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            try:
                await asyncio.sleep(1)
                # Send current active states periodically to keep connection alive
                active_ids = list(TickingState.global_publish_log.keys())
                msg = {"type": "tick", "tick": int(time.time()*1000), "active": active_ids, "tree_version":tree_version}
                await ws.send_text(json.dumps(msg))
            except (RuntimeError, WebSocketDisconnect):
                # Client disconnected, break the loop
                break
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        clients.discard(ws)


@app.get("/api/history",summary="work in progress")
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


