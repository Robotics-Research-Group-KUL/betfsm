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


## Event receivers

|    Event receiver     | Description                                                           |
|-----------------------|-----------------------------------------------------------------------|
|  [TopicEventReciever][betfsm_ros.TopicEventReceiver]   | Receives events from a ROS2 topic and uses a FIFO queue to <br/>store the events. |
|  [Ctrl_C_Receiver][betfsm.Ctrl_C_Receiver]       | Rembers whether ctrl-c has been pressed such that the polling <br/>function can check for that event.   |
|  [HTTPEventReceiver][betfsm.HTTPEventReceiver] | Checking for events coming from the webserver, to implement <br/>GUI interaction. |




!!! TODO

    Investigate Jobqueue idea:  way to push jobs to a queue together with a piece of blackboard in payload.
    An Event that consumes it and puts the payload in the blackboard. Different types of queues


## Conditions

Conditions are classes that are used for which events to listen.  Conditions can **consume** their
event, i.e. if immediately called again, they will not return the same event again, or they
do **not consume** the event.  For a queue related event, this corresponds to taking away
the event from the queue or to leave it in the queue.

Conditions **return a string** corresponding to the matching "event". For a condition coming in via HTTP
or a ROS2 topic, this is the string that is communicated.  For conditions such as Timeout_Condition
or Callback_Condition, it is a string that is given in the constructor arguments. If there is no event,
a condition **returns None**.

Conditions can be composed using operators:  **|** (corresponding to OR) and **&** (corresponding to AND)
There is a [Not][betfsm.Not] condition, that however requires an additional  string as constructor argument
since it needs to know what to return if the underlying Condition returns None.

So, Conditions can form a small expression tree that is evaluated at run time. For debugging purposes
you can convert it to a string using `str(my_condition_obj)`.

The HTTP events are managed by the (asynchronous) webserver that is also dealing with the visualization.
It has an API documentation using Swagger under `0.0.0.0:8000/docs` (or the endpoint that you indicated
in the command line)

| Condition              | description                                                          |
|------------------------|----------------------------------------------------------------------|
| Condition              | Base class for conditions                                            |
| [Not][betfsm.Not]      | Negation                                                             |
| [OrCondition][betfsm.OrCondition] | OR, typically you use `|` operator                         |
| [AndCondition][betfsm.AndCondition] | AND, typically you use `&` operator                      |
| [Callback_Condition][betfsm.Callback_Condition] | a condition determined by a callback function|
| [Timeout_Condition][betfsm.Timeout_Condition] | a Timeout condition (can be repeated or one-shot) |
| [HTTPEvent_Condition][betfsm.HTTPEvent_Condition]| A string is received on an HTTP event channel (see HTTP API of BeTFSM) |
| [Ctrl_C_Condition][betfsm.Ctrl_C_Condition] | Ctrl-C has been pressed  |
| [TopicEvent_Condition][betfsm_ros.TopicEvent_Condition] | A string is received on a ROS2 Topic |




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




## Usage patterns


- Configure the polling callback function. You pass the function name, you do not call polling_func yourself.
  ```python
    cond = HTTPEvent_Condition("betfsm")
  ```

- Check whether there is an event, if not immediately returns SUCCEED, if e_finished return "FINISH"
Probabily you want to call this in a loop, make sure that there is at least one yield TICKING in the loop! (**does not wait**)
  ```python 
    EventOutcome( "check_finalized", 
                    cond, 
                    { NO_EVENT:SUCCEED, "e_finished":"FINISH" }
    )
  ``` 

- **Waits** until it receives "e_open" or "e_finished". If "e_open" is received return "OPEN" outcome,
if "e_finished" is received, return SUCCEED outcome.
  ```python 
    EventOutcome( "check_finalized",  cond, { 
                    "e_open" :"OPEN", 
                    "e_finished":SUCCEED 
                 })
  ``` 
equivalent to:
  ```python 
    EventOutcome( "check_finalized",  cond, { 
                    NO_EVENT : TICKING,
                    "e_open" :"OPEN", 
                    "e_finished":SUCCEED 
                 })
  ``` 
- Executes nominal_subtree, if "e_something_is_wrong" is received, interrupts nominal subtree and directly returns CANCEL.
If "e_open_gripper" is received executes opengripper_subtree concurrently with nominal_subtree until
both return SUCCEED or one of them returns something else (i.e. ConcurrentSequence behavior).
  ```python 
      EventConcurrent( "check_finalized",  cond,  { 
                        NO_EVENT:                nominal_subtree, 
                        "e_something_is_wrong" : AlwaysOutcome(CANCEL), 
                        "e_open_gripper":        open_grippersubtree 
                })
  ``` 

- Executes a *nominal_subtree* that already checks for the "e_finished" event. We augment
  this behavior from the 'outside' with checking for "e_open_gripper" and "e_something_is_wrong" 
  (as described above). Events can be checked at multiple places.  If the *nominal_subtree*
  is finished but *open_gripper_subtree* is still running, it will wait for *open_gripper_subtree*.

  The "e_open_gripper" and "e_something_is_wrong" event strings can also come via an HTTP connection.

  ```python
    cond_topic = TopicEvent_Condition()
    cond_http  = HTTPEvent_Condition()

    nominal_subtree = Sequence("crospi_statemachine, [ ... EventOutcome("check_finished", cond_topic])....])
    EventConcurrent( "add_error_check",  cond_topic | cond_http, { 
                    NO_EVENT: nominal_subtree,
                    "e_something_is_wrong":AlwaysOutcome(CANCEL),
                    "e_open_gripper":      open_gripper_subtree                    
                 })
  ```

-            
    The following executes a nominal state machine `nominal_sm`.  If a ctrl-c is pressed or a STOP is send over HTTP or after 60 seconds, 
    deactivates Crospi, and interrupt the nominal state machine and return "CANCEL".  

    Every 15 seconds a time-out condition is triggered and the nominal state machine is interrupted and a message is send out.    
    ```
    sm = EventSequential("check", 
                         Ctrl_C_Condition("CTRL_C") | HTTPEvent_Condition() | Timeout_Condition("MESSAGE",15,-1) | Timeout_Condition("QUIT",60),
                         event_map={
                            NO_EVENT: nominal_sm,
                            "CTRL_C": CrospiDeactivate(force_outcome="CANCEL"),
                            "STOP":   CrospiDeactivate(force_outcome="CANCEL"),
                            "MESSAGE" : say_message,
                            "QUIT" :  CrospiDeactivate(force_outcome="CANCEL")
                        }
    )
    ```
    Note that you have to create three times the CrospiDeactivate(...).  Otherwise your BeTFSM graph wouldn't be a **tree** anymore. (This condition is checked).

    !!! TODO
        In principle, since EventSequential executes all non-NO_EVENT state machines sequentially, an implementation could handle multiple identical
        state machines as values for the event_maps. This is not (yet) done. This is **not** the case for EventConcurrent, where it is really necessary.
