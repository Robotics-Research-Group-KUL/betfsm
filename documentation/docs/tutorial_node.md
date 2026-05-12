## Defining your own BeTFSM node

**BeTFSM** is about orchestrating and coordinating different subtasks in a scalable way. Of course the subtasks first have
to be defined before they can be coordinated. These subtasks are represented by nodes in the BeTFSM tree.  You can define
these yourselves or you can use pre-defined nodes. Here we discuss how to define your own node to connect BeTFSM to your
functionality.  One of the advantages of BeTFSM is that these nodes can be very easily defined.

As a simple example we create our own BeTFSM node that implements a countdown:
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

