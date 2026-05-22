## Handling interruption

Especially when dealing with the physical world, such as in robotics, it becomes important
to deal with unexpected events and interruptions. In BeTFSM, these things are dealt with
using [events](event.md).  This tutorial gives a basic introduction to one of the more
basic type of event, a user pressing Ctrl-C.  It is probably not a good idea to
let the robot continue its current task when the BeTFSM coordinating these robots
tasks is interrupted by the default Ctrl-C handler of Python.

In BeTFSM, this is handled by **EventReceiver**, typically singleton classes, that 
receive events and expose them to BeTFSM.  **Conditions** that check if one of the
listed events occurred and return the corresponding event string. And **Event checkers**
that coordinate everything with the behavior tree and handle the event concurrently, sequentially
or by generating a specific outcome.


Explanation of the code:
- From line 13-26, we define a custom node, that at each tick counts down until the
  counter reaches zero and then returns SUCCEED.

- Line 30-37, we define our nominal "application", in this case just a sequence that uses
  our Countdown node.

- Line 40-52 defines the cleanup procedure in case of interruption. Instead of the
  [TimedWait][betfsm.TimedWait] node, here we use [EventOutcome][betfsm.EventOutcome]
  to achieve equivalent behavior.  Its second argument is a Timeout_Condition that will
  fire the event string "DONE" 5 seconds after it is started.  In the event_map we specify
  what should happen when "DONE" arrives, in this case give out SUCCEED outcome.

- Line 66-69 define the handling of Ctrl-C events. Here [EventSequential][betfsm.EventSequential]
  is used. Its event_map maps event strings to not to outcome strings but whole BeTFSM subtrees.
  [EventSequential][betfsm.EventSequential] does interrupt the nominal subtree (specified with NO_EVENT),
  but the evaluation is switched over to another subtree when an event string comes in. (and in the process
  pauses the nominal statemachine).  

    - When **any** of the subtrees return something different from SUCCEED or TICKING,
    [EventSequential][betfsm.EventSequential] returns with that outcome. 
    - It also returns when**all** subtrees returned SUCCEED.  In other words, if the nominal 
    subtree finishes with SUCCEED, [EventSequential][betfsm.EventSequential]
    will still wait until the cleanup subtree is finished.  
    - The cleanup subtree can also decide to  will continue running the paused nominal 
    subtree by returning SUCCEED.

Running of the code:

  - When you run the code, you can click on the link that is printed in the console, this gives you a 
   graphical visualization of what going on.
  - See what happens when you press Ctrl-C
  - Try to press Ctrl-C within 5 seconds of the end, can you predict what happens?
  - Change [EventSequential][betfsm.EventSequential] to [EventConcurrent][betfsm.EventConcurrent], and press
   Ctrl-C, can you predict what happens?

```python linenums="1"
--8<-- "betfsm/betfsm_examples/example_sequence3.py"
```

