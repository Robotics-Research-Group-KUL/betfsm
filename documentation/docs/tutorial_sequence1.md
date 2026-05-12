## Your first BeTFSM behaviour tree

This tutorial builds your first very simple BeTFSM behaviour tree. This tree will display a start message, countdown from 4 and displays an end message.  You can find the code of the example in ```betfsm_examples``` in the files ```example_sequence1.py```, ```example_sequence2.py``` and ```example_sequence3.py```.  This tutorial uses the BeTFSM node defined in the [My first BetTFSm node](tutorial_node.md) tutorial.

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

These alternative ways to define a subtree are very useful.  The **first** approach, an all inclusive call to the constructror, gives a one statement quick definition of a whole tree. The **second**, a gradual built-up of the subtrree, allows to write code that constructs the tree (e.g. from a 
higher-level plan).  The **third** approach defines a new class, and allows to encapsulate the tree and further parameterize it by adding parameters to the constructor; from then on the class can be used as
just another BeTFSM node.

All of these alternatives result in the same BeTFSM tree.





### Running your BeTFSM tree

To execute the BeTFSM tree, you use a BeTFSM runner.  Multiple BeTFSM runners exist, e.g. to interface with a graphical user interface while executing or to use ROS2 primitives for timing.  In this example we will use the pure python BeTFSM runner [Runner][betfsm.Runner] or the ROS2-version [ROSRunner][betfsm_ros.ROSRunner]

You pass the BeTFSM tree `sm`, the (empty) blackboard, and the desired frequency to the constructor and call the ```run``
method to run the BeTFSM tree.  It will return the outcome of the BeTFSM tree.

``` py
    runner = Runner(sm, bb, frequency=100.0,debug=True) # Hz
    outcome = runner.run()
```
For this example, also the debug-mode is turned on, such that it is more clear what happens in each tick. For a real applications this debug log lines are typically not turned on, since they pollute the log in an extreme way.


## Putting everything together

Below you find the whole implementation. This implementation can also be found in the ```betfsm_examples``` directory in the file ```example_sequence1.py``` ( or ```example_sequence2.py``` or ```example_sequence3.py``` for the alternate ways of specifying the sequence).


```python linenums="1"
--8<-- "betfsm/betfsm_examples/example_sequence1.py:2:"
```

To execute this, you can run:
```
    cd ./betfsm_examples
    python example_sequence1.py
```

It will return a log trace similar to:

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



 

