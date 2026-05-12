## Overview of the API


The API is split up in three parts:

  - **BeTFSM**: pure Python, main framework for Behavior-tree based, ticking, finite state machines.
  - **BeTFSM ROS2**": ROS2 related part
  - **BeTFSM cROSpi**: cROSpi related part

### BeTFSM

- Fundamental classes:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| TickingState             | Fundamental base-class that contains the main logic where all the<br> other states are based on | 
| Generator                 | Auxiliary class using Python generator, i.e. yield, to facilitate definition<br> of nodes.  To be used as a base class |
| GeneratorWithState        | Auxiliary class inherited from Generator that also manages a state |
| GeneratorWithList         | Auxiliary class inherited from Generator that also manages a list of states |
| TickingStateMachine       | Auxiliary class to implement state machines, just have to define the states<br> and the transitions | 

- Orchestration:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [Sequence][betfsm.Sequence]  |  Executes sequentially its children as long as they return SUCCEED<br> or TICKING, any other outcome returns directly with that outcome |
| [Fallback][bestfsm.Fallback]                  |  Executes sequentially its children as long as they return CANCEL<br> or TICKING, any other outcome returns directly with that outcome (including SUCCEED) |
| [Adapt][betfsm.Adapt] | Adapts the outcome of a single child using a transition table |
| While                  |   |
| Repeat                  |   |



- Concurrency:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [ConcurrentSequence][betfsm.ConcurrentSequence] | |
| [ConcurrentFallback][betfsm.ConcurrentFallback] | |
| [Concurrent][betfsm.Concurrent] | |


- Utility:

| Class                     |      Description            |
|---------------------------|-----------------------------|
















!!! TODO

    finish these tables