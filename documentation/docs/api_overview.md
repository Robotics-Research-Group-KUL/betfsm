# Overview of the API


The API is split up in three parts:

  - **BeTFSM**: pure Python, main framework for Behavior-tree based, ticking, finite state machines.
  - **BeTFSM ROS2**": ROS2 related part
  - **BeTFSM cROSpi**: cROSpi related part



!!! TODO

    finish these tables

## BeTFSM

### Fundamental classes:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [TickingState][betfsm.TickingState]             | Fundamental base-class that contains the main logic where all the<br> other states are based on | 
| [Generator][betfsm.Generator]                 | Auxiliary class using Python generator, i.e. yield, to facilitate definition<br> of nodes.  To be used as a base class |
| [GeneratorWithState][betfsm.GeneratorWithState]        | Auxiliary class inherited from Generator that also manages a state  |
| [GeneratorWithList][betfsm.GeneratorWithList]         | Auxiliary class inherited from Generator that also manages a list of states |
| [TickingStateMachine][betfsm.TickingStateMachine]       | Auxiliary class to implement state machines, just have to define the states<br> and the transitions | 



### Orchestration:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [Sequence][betfsm.Sequence]  |  Executes sequentially its children as long as they return SUCCEED<br> or TICKING, any other outcome returns directly with that outcome |
| [Fallback][betfsm.Fallback] |  Executes sequentially its children as long as they return CANCEL<br> or TICKING, any other outcome returns directly with that outcome<br> (including SUCCEED) |
| [Adapt][betfsm.Adapt] | Adapts the outcome of a single child using a transition table |
| [While][betfsm.While]                  |   |
| [Repeat][betfsm.Repeat]                  |   |


### Concurrency:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [ConcurrentSequence][betfsm.ConcurrentSequence] | |
| [ConcurrentFallback][betfsm.ConcurrentFallback] | |
| [Concurrent][betfsm.Concurrent] | |

### Timing:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [TimedWait][betfsm.TimedWait]     | |
| [TimedRepeat][betfsm.TimedRepeat] | |




### Utility:

| Class                     |      Description            |
|---------------------------|-----------------------------|
| [Message][betfsm.Message] | |
| [LogBlackboard][betfsm.LogBlackboard] | |
| [Adapt][betfsm.Adapt] | | 
| [Compute][betfsm.Compute] | |
| [AlwaysOutcome][betfsm.AlwaysOutcome]  ||
| [WaitFor][betfsm.WaitFor] | |
| [WaitForever][betfsm.WaitForever] | redundant AlwaysOutcome(TICKING) Waitfor(lambda bb: True| 





## BeTFSM ROS

### Timing related

| Class                     |      Description            |
|---------------------------|-----------------------------|
| TimedRepeat for ROS2| |
| TimedWait for ROS2 | |
| Timeout | |


### Services and actions

| Class                     |      Description            |
|---------------------------|-----------------------------|
| ServiceClient | Creates a TickingState that calls a ROS2 service and generates an outcome when the service <br>returns back. While waiting, it continues to tick |
| ActionClient  | Creates a TickingState that calls an action and generates an outcome when the action returns <br>back.  While waiting, it gets the response of the action to the blackboard  and returns TICKING. |
| LifeCycle | Manages the life cycle of another node|



## BeTFSM cROSpi

| Class                     |      Description            |
|---------------------------|-----------------------------|
| load_task_list ||
| setTaskParameters | |
| ReadRobotSpecification | |
| ReadTaskSpecification | |
| eTaSLOutput | |
| eTaSLEvent | |
| eTaSL_StateMachine | |







