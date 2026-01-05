## Your first BeTFSM behaviour tree

This tutorial builds your first very simple BeTFSM behaviour tree. This tree will display a start message, countdown from 4 and displays an end message.  You can find the code of the example in ```betfsm_examples``` in the files ```example_sequence1.py```, ```example_sequence2.py``` and ```example_sequence3.py```.

### Defining your own BeTFSM node

We start by creating our own BeTFSM node that implements a countdown:
``` py
class CountDown(Generator):
    """
    A simple generator state that counts down from a given number.
    """
    def __init__(self, name, count):
        super().__init__(name, [SUCCEED])
        self.count = count

    def co_execute(self, blackboard):
        for i in range(self.count, 0, -1):
            get_logger().info(f"{self.name}: {i}")
            yield TICKING
        get_logger().info(f"{self.name}: Finished counting down!")
        yield SUCCEED

```

A BeTFSM node is a python class that inherits from [TickingState][betfsm.betfsm.TickingState].  However, to simplify implementation, we mostly inherit from the subclass [Generator][betfsm.betfsm.Generator].  This sub-class uses [Python generators](https://www.geeksforgeeks.org/generators-in-python/) to simplify implementation. 

To facilitate definitions of BeTFSM nodes, BeTFSM uses synchronous execution for cooperative multi-tasking, this avoids to deal with thread management, mutexes, etc.  
The whole tree executes at **a fixed sample rate** and at each sample interval ('tick'), a node either returns TICKING (indicating that the node is not yet finished and wants to be called back the next tick) or another outcome (such as SUCCEED).  All outcomes are strings.  If really necessary, a BeTFSM node can still spawn and *manage* its own thread(s), but this is internal to the node and should not concern the framework.

When you inherit from [Generator](betfsm.betfsm.Generator), 
the main implementation of functionality is in the method `co_execute` (the 'co' refers to co-routine), you can use python's yield to return control back to the system and pass the outcome of the state, the next tick control will resume at the same location in `co_execute` (or go to the next state when the outcome is different from SUCCEED).

!!! Warning
    A TickingState should return within a time period that is significantly smaller than the sample rate. Please take into account that multiple nodes can be executed simultaneously (i.e. within the same tick)

Each BeTFSM node needs to have a **name**, this name identifies the node and should be different for all instances of the node.
Typically the name is either passed to the constructor or is auto-generated, e.g. with some sequence number.  To guard against programmatic errors or misspelling of outcomes, a BeTFSM node also **declares it possible outcomes** and checks these. There is no need to declare the outcome TICKING.

The ```co_execute`` method also gets passed a ```blackboard``` parameter.  This is a (hierarchical) python dictionary that nodes can use to exchange information.

### Putting your BeTFSM tree together

BeTFSM contains a library of pre-defined nodes that implement the typical Behaviour-tree nodes such as [Sequence][betfsm.betfsm.Sequence] or [Fallback][betfsm.betfsm.Fallback].

As said before, this tutorial builds your first very simple BeTFSM behaviour tree. This tree will display a start message, countdown from 4 and displays an end message.


For this example, we will use the node [Message][betfsm.betfsm.Message] from the BeTFSM library. This nodes prints a given message to the log.

We will use a [Sequence][betfsm.betfsm.Sequence] to execute first the start message, then the count down and then the end message.  A [Sequence][betfsm.betfsm.Sequence] executes the first element in the sequence and continues with this element as long as it returns TICKING, and the sequence itself also returns TICKING.  If the elements returns SUCCEED, it continues with the second element in the sequence (and the sequence does not yet return anything). If the element returns another outcome, the sequence returns that outcome.

``` py
    sm = Sequence("sequence_phase", [
        Message(msg="--- Starting Sequence Phase ---"),
        CountDown("seq_counter_1", 4),
        Message(msg="--- Sequence Phase Finished ---")
    ])
```

Here the sequence is passed in the constructor. An alternative is to build up the sequence with
method calls:
``` py
    sm = Sequence("sequence_phase")
    sm.add_state( Message(msg="--- Starting Sequence Phase ---") )
    sm.add_state( CountDown("seq_counter_1", 4) )
    sm.add_state( Message(msg="--- Sequence Phase Finished ---") )
```

Another alternative is to encapsulate the sequence as a separate class. This is useful when
the sequence itself needs to be reusable, with some additional parameters passed in the constructor:

``` py
class MySequence(Sequence):
    def __init__(self, count):
        super().__init__("MySequence") # do not forget to initialize the super class
        self.count = count
        self.add_state( Message(msg="--- Starting Sequence Phase ---") )
        self.add_state( CountDown("seq_counter_1", self.count) )
        self.add_state( Message(msg="--- Sequence Phase Finished ---") )



```

This last can be useful in a scenario where you want to specify a BeTFSM-tree that handles a pick-up operation together gripper operations and error-handling, parameterized in e.g. pick-up and drop locations.

All of these alternatives result in the same BeTFSM tree.





### Running your BeTFSM tree

To execute the BeTFSM tree, you use a BeTFSM runner.  Multiple BeTFSM runners exist, e.g. to interface with a graphical user interface while executing or to use ROS2 primitives for timing.  In this example we will use the pure python BeTFSM runner [BeTFSMRunner][betfsm.betfsm.BeTFSMRunner].

You pass the BeTFSM tree `sm`, the (empty) blackboard, and the desired frequency to the constructor and call the ```run``
method to run the BeTFSM tree.  It will return the outcome of the BeTFSM tree.

``` py
    runner = BeTFSMRunner(sm, bb, frequency=100.0,debug=True) # Hz
    outcome = runner.run()
```
For this example, also the debug-mode is turned on, such that it is more clear what happens in each tick. For a real applications this debug log lines are typically not turned on, since they pollute the log in an extreme way.


## Putting everything together

Below you find the whole implementation. This implementation can also be found in the ```betfsm_examples``` directory in the file ```example_sequence1.py``` ( or ```example_sequence2.py``` or ```example_sequence3.py``` for the alternate ways of specifying the sequence).

``` py
# Example of a sequence
#!/usr/bin/env python3

import time
from betfsm.betfsm import (
    BeTFSMRunner, Sequence,  
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard
)
from betfsm.graphviz_visitor import to_graphviz_dotfile
from betfsm.logger import get_logger


# A user defined TickingState:
class CountDown(Generator):
    """
    A simple generator state that counts down from a given number.
    """
    def __init__(self, name, count):
        super().__init__(name, [SUCCEED])
        self.count = count

    def co_execute(self, blackboard):
        for i in range(self.count, 0, -1):
            get_logger().info(f"{self.name}: {i}")
            yield TICKING
        get_logger().info(f"{self.name}: Finished counting down!")
        yield SUCCEED

def main():
    # Create a blackboard
    bb = {}

    # 1. Example Sequence
    # This will execute its children one after another.
    sm = Sequence("sequence_phase", [
        Message(msg="--- Starting Sequence Phase ---"),
        CountDown("seq_counter_1", 4),
        Message(msg="--- Sequence Phase Finished --- (message 1 of 2)"),
        Message(msg="--- Sequence Phase Finished --- (message 2 of 2)")
    ])

    # 3. Run it using BeTFSMRunner at 100 Hz
    runner = BeTFSMRunner(sm, bb, frequency=100.0,debug=True) # Hz
    get_logger().info("Running State Machine... (explicitly logged in main)")
    outcome = runner.run()
    get_logger().info(f"State Machine Finished with outcome: {outcome} (explicitly logged in main)")
    
if __name__ == "__main__":
    main()
```

To execute this, you can run:
```
    cd ./betfsm_examples
    python example_sequence1.py
```

It will return the following log trace:

``` 
INFO : Running State Machine... (explicitly logged in main)
DEBUG : BeTFSMRunner time: 129055.750 s started (frequency:100.0)
INFO : --- Starting Sequence Phase ---
INFO : seq_counter_1: 4
DEBUG : 129055.750 s : looping 
INFO : seq_counter_1: 3
DEBUG : 129055.760 s : looping 
INFO : seq_counter_1: 2
DEBUG : 129055.770 s : looping 
INFO : seq_counter_1: 1
DEBUG : 129055.780 s : looping 
INFO : seq_counter_1: Finished counting down!
INFO : --- Sequence Phase Finished --- (message 1 of 2)
INFO : --- Sequence Phase Finished --- (message 2 of 2)
DEBUG : 129055.790 s : looping 
INFO : State Machine Finished with outcome: succeeded (explicitly logged in main)
```
This log also contains two log lines that where generated by explicit log statements in the main().
The DEBUG lines indicate the time that a new tick starts.
Note that the first message and the first step of the counter are executed in the same tick.  The
last step in the counter and two messages after it are also executed in one tick.
[Sequence][betfsm.betfsm.Sequence] is greedy in that sense that it tries to execute as much elements as possible
in one tick, until one of the elements returns TICKING or something else than SUCCEED.

If you go to [Sequence][betfsm.betfsm.Sequence] (or to many of the more fundamental BeTFSM nodes),
you'll see a diagram that indicates what happens at each tick.  These diagrams can be very useful to 
understand the exact semantics.

 

