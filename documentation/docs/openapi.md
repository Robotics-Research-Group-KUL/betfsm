# BeTFSM webserver API

_Version: 0.1.0_


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




## Table of Contents

- [`GET` /api/event_list](#send-event-list-api-event-list-get)
- [`POST` /api/event](#receive-event-api-event-post)
- [`GET` /api/blackboard/{path}](#get-blackboard-value-api-blackboard-path-get)
- [`PUT` /api/blackboard/{path}](#set-blackboard-value-api-blackboard-path-put)
- [`GET` /api/alive](#alive-api-alive-get)
- [`GET` /api/tree](#get-tree-api-tree-get)
- [`GET` /api/history](#get-history-api-history-get)


## Endpoints


### /api/event_list

***GET***

**Summary:** returns lists of events used by BeTFSM


A dictionary that maps a type of condition list of events used *somewhere* by that condition.  This is a static 
description, generated during construction.  This are lists of events **accessible** by the condition, not only
the events that this type of condition actually generates.


**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` | a dictionary that maps BeTFSM Condition Classes to a list of events. |

**Response Examples — `200`**

### /api/event

***POST***

**Summary:** Push an event towards a HTTPEventReceiver

**Request Body:**

| Media type | Schema |
| ---------- | ------ |
| `application/json` | `#/components/schemas/Event` |

**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` |  |
| `422` | Validation Error |


### /api/blackboard/{path}

***GET***

**Summary:** Get a value from the blackboard


Returns the value stored at the given hierarchical path.  

- Each path component is separated with '/'. 
- List elements are adressed by numeric indices, e.g. /output/0/name
- length is obtained using '~len', e.g. /output/~len
        

**Parameters:**

| Name | In | Required | Type | Description |
| ---- | -- | -------- | ---- | ----------- |
| `path` | path | yes | string |  |

**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` | Successful Response |
| `422` | Validation Error |


***PUT***

**Summary:** Set a value in the blackboard


Stores a value at the given hierarchical path, creating intermediate nodes if needed.
A path and a value should be given.
The path has the following syntax:

- '~append' appends a new value to a list
- '~insert:N' inserts a value at index N, e.g. /output/~insert:2
- "~del:N" deletes a value at index N
- "~pop~ removes the last element
- "N" replaces the element at index N


**Parameters:**

| Name | In | Required | Type | Description |
| ---- | -- | -------- | ---- | ----------- |
| `path` | path | yes | string |  |

**Request Body:**

| Media type | Schema |
| ---------- | ------ |
| `application/json` | `#/components/schemas/SetValueRequest` |

**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` | Successful Response |
| `422` | Validation Error |


### /api/alive

***GET***

**Summary:** Ping server to see if it is alive

**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` | Successful Response |


### /api/tree

***GET***

**Summary:** Get the BeTFSM tree in JSON

**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` | Successful Response |


### /api/history

***GET***

**Summary:** work in progress

**Parameters:**

| Name | In | Required | Type | Description |
| ---- | -- | -------- | ---- | ----------- |
| `from_ts` | query | yes | number |  |
| `to_ts` | query | yes | number |  |

**Responses:**

| Code | Description |
| ---- | ----------- |
| `200` | Successful Response |
| `422` | Validation Error |


## Schemas

#### Event

| Property | Type | Required | Description |
| -------- | ---- | -------- | ----------- |
| `channel` | string | no | Name of the channel, i.e. a BeTFSM receiver subscribes to channel and will have a queue of all events for that channel. |
| `event` | string | no | Name of the event |

#### HTTPValidationError

| Property | Type | Required | Description |
| -------- | ---- | -------- | ----------- |
| `detail` | array<#/components/schemas/ValidationError> | no |  |

#### SetValueRequest

| Property | Type | Required | Description |
| -------- | ---- | -------- | ----------- |
| `value` |  | yes | The value to store at the given path. Can be any valid JSON type. |

#### ValidationError

| Property | Type | Required | Description |
| -------- | ---- | -------- | ----------- |
| `loc` | array<string> | yes |  |
| `msg` | string | yes |  |
| `type` | string | yes |  |





## Acknowledgements

Generated by [openapi-to-markdown](https://thetexttool.com/tools/openapi-to-markdown#google_vignette) and manually edited for style.