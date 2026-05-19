# Events in BeTFSM


Without loss of generality, **events** in BeTFSM are identified by strings.  If the event source uses another
form of event identification, the event receiver (see below) will translate this into strings.

Event-handling in BeTFSM is handled in two parts:

 - An **event receiver** that receives events.  Most often this receiver should be a **singleton class** for each channel by which 
   you receive the events.  It is up to the event receiver to decide on the policy of receiving the events, e.g. whether they should
   be queued in a FIFO way, a priority queue or something else.  The event receiver should provide a **polling function**, a
   function that takes a list of events and returns a matching event that has been received or None if no there is no matching
   event received.  An event receiver should provide a **consuming** (i.e. the matching event is removed from the queue) and
    **non-consuming** (i.e. the matching event is not removed from the queue) polling function.  Not all event receivers have or should 
    have a queue (e.g. ctrl-C handler).  
 - An **event checker** is a TickingState that can be used in a BeTFSM tree to check whether an event has been received. The modalities
   such as blocking/one-time check  or sending out an outcome/executing a BeTFSM subtree/interrupting a BeTFSM tree depend on the
   specific event checker class. The event checker and event receiver are (very) loosely coupled using a **callback function**
   given as an argument to the event checker constructor. Typically a very small function couples the event receiver class and 
   polling function with the blackboard and is used as callback function.

??? Note "**Checking at multiple locations in the BeTFSM tree**"
    A big advantage of the decoupling of event receiver with event checker is that you can check the events
    at multiple locations in your BeTFSM tree.  It is not a problem to have a consuming check at multiple locations
    in your BeTFSM tree, as long as you check for different events.  If you want to check for the same event at
    different locations, you probably want only to have one check consuming and the other non-consuming.


??? Note "**Queuing at BeTFSM's side**"
      The event receiver has to manage its own queue and cannot easily let the middleware handle it.  It needs
      to examine the whole queue for a matching event.

??? Note "**Multiplexing event receivers**"
      For particular applications, such as listening to multiple topics, a short, special purpose callback function can
      be written to call multiple event receivers and fuse them together according to your application specific policies.

## Event receivers

|    Event receiver     | Description                                                           |
|-----------------------|-----------------------------------------------------------------------|
|  [TopicEventReciever][betfsm_ros.TopicEventReceiver]   | Receives events from a ROS2 topic and uses a FIFO queue to <br/>store the events. |
|  [Ctrl_C_Receiver][betfsm.Ctrl_C_Receiver]       | Rembers whether ctrl-c has been pressed such that the polling <br/>function can check for that event.   |
|  *to be implemented*  | Checking for events coming from the webserver, to implement <br/>GUI interaction. |


!!! TODO
    
    What about timing events?

!!! TODO

    More structured way besides blackboard with input and output parameters?

## Event checkers

| Event checker         | description                                                           |
|-----------------------| ----------------------------------------------------------------------|
| [EventOutcome][betfsm.EventOutcome]          | A BeTFSM node that checks for a given list of events and gives an <br> outcome depending on which event is received. Can be blocking or<br> non-blocking|
| [EventConcurrent][betfsm.EventConcurrent]          | Similar as a ConcurrentSequence, but the children are only started <br> when an event is received. Multiple children can be started<br> concurrently. The same child can be started multiple times but will<br> only be executed sequentially. The NO_EVENT child is active from<br> the start and will only executed once (i.e. it is a *nominal subtree*)|
| [EventSequential][betfsm.EventSequential] | Similar as sequence, but children are started sequentially, one at a <br>time, when an event is received. Children are started one at a time.  <br>The same child can be  started multiple times. The NO_EVENT child<br> is active from the start and will only be executed once (i.e. it is a <br>*nominal subtree*)|


[EventOutcome][betfsm.EventOutcome], [EventConcurrent][betfsm.EventConcurrent] and [EventSequential][betfsm.EventSequential] take an **event_map** as constructor argument. An event_map
maps an event string to a BETFSM outcome (also a string)( [EventOutcome][betfsm.EventOutcome] ) or a BeTFSM subtree (one of the TickingState subclasses) ( [EventConcurrent][betfsm.EventConcurrent], [EventSequential][betfsm.EventSequential] ).

[AlwaysOutcome][betfsm.AlwaysOutcome] is useful when you want to mix returning directly an outcome and subtrees.


A typical use case for [EventConcurrent][betfsm.EventConcurrent] is handling events that have to run concurrently with the nominal subtree specified using "NO_EVENT" in the event_map, such as "open gripper"
A typical use case for [EventSequential][betfsm.EventSequential] is handling a cancel event that directly interrupts the nominal subtree and starts executing a cleanup subtree, such as "soft_emergency_stop"



## Polling functions

Polling functions bind event checkers with event receivers.  The following
functions are factory functions that return polling_func functions,
they are not polling_func's themselves.

| Polling function                                         | description                                        |
|----------------------------------------------------------|----------------------------------------------------|
| [crospi_polling_func][betfsm_crospi.crospi_polling_func]  |  to read out events from a topic (using crospi conventions) |
| [blackboard_polling_func][betfsm.blackboard_polling_func] |  to read out events from a blackboard location |
| [ctrl_c_polling_func][betfsm.ctrl_c_polling_func]         |  to read out ctrl-c pressed events  |
| [combine][betfsm.combine]                                 |  combines multiple callbacks |
## Usage patterns


- Configure the polling callback function. You pass the function name, you do not call polling_func yourself.
  ```python
    pollingfunc = crospi_polling_func()
  ```

- Check whether there is an event, if not immediately returns SUCCEED, if e_finished return "FINISH"
Probabily you want to call this in a loop, make sure that there is at least one yield TICKING in the loop!.
  ```python 
    EventOutcome( "check_finalized", 
                    polling_func, 
                    { NO_EVENT:SUCCEED, "e_finished":"FINISH" }
    )
  ``` 

- Waits until it receives "e_open" or "e_finished". If "e_open" is received return "OPEN" outcome,
if "e_finished" is received, return SUCCEED outcome.
  ```python 
    EventOutcome( "check_finalized",  polling_func, { 
                    "e_open" :"OPEN", 
                    "e_finished":SUCCEED 
                 })
  ``` 


- Executes nominal_subtree, if "e_something_is_wrong" is received, directly returns CANCEL.
If "e_open_gripper" is received executes opengripper_subtree concurrent with nominal_subtree until
both return SUCCEED or one of them returns something else (i.e. ConcurrentSequence behavior).
  ```python 
      EventSubtree( "check_finalized",  polling_func,  { 
                        NO_EVENT:                nominal_subtree, 
                        "e_something_is_wrong" : AlwaysOutcome(CANCEL), 
                        "e_open_gripper":        open_grippersubtree 
                })
  ``` 

- Executes a *nominal_subtree* that already checks for the "e_finished" event. We augment
  this behavior from the 'outside' with checking for "e_open_gripper" and "e_something_is_wrong" 
  (as described above). Events can be checked at multiple places.  If the *nominal_subtree*
  is finished but *open_gripper_subtree* is still running, it will wait for *open_gripper_subtree*  

  ```python
    nominal_subtree = Sequence("crospi_statemachine, [ ... EventOutcome("check_finished", crospi_polling_func])....])
    EventSubtree( "add_error_check",  polling_func, { 
                    NO_EVENT: nominal_subtree,
                    "e_something_is_wrong":AlwaysOutcome(CANCEL),
                    "e_open_gripper":      open_gripper_subtree                    
                 })
  ```

- Remember the behavior similar to ConcurrentSequence: All children of EventSubtree need to 
   return SUCCEED (if they are activated), so in the following below, the event "e_nothing_done" 
   doesn't do anything.  This behavior is deliberately chosen such that you can execute subtrees similar
   to open_gripper_subtree simultaneously with the nominal_subtree and you have a guarantee that the trees
   will be finished, except when someone returns something else than SUCCEED. So if nominal_subtree is finished
   and "e_open_gripper" was called, EventSubtree will wait until open_gripper_subtree is finished.

  ```python
    nominal_subtree = Sequence("crospi_statemachine, [ ... EventOutcome("check_finished", crospi_polling_func])....])
    EventSubtree( "add_error_check",  polling_func, { 
                    NO_EVENT: nominal_subtree,
                    "e_something_is_wrong":AlwaysOutcome(CANCEL),
                    "e_open_gripper":      open_gripper_subtree                    
                    "e_nothing_done":      AlwaysOutcome(SUCCEED)
                 })
  ```
