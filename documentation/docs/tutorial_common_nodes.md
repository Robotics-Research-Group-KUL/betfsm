## An example with common BeTFSM nodes

This example is an example of everything together. It contains many of the common
BeTFSM nodes and demonstrates how to use them.  This example is not explained
in detail, but for further information, you can always look up the details in this documentation. 

It defines a few BeTFSM nodes such as ```CountDown```, ```CountDownWithDelay``` and ```CancelingState```.  Theses nodes are then composed using behaviour-tree nodes such as [Sequence][betfsm.betfsm.Sequence], [ConcurrentSequence][betfsm.betfsm.ConcurrentSequence], [Concurrent][betfsm.betfsm.Concurrent], [TimedRepeat][betfsm.betfsm.TimedRepeat] and [Fallback][betfsm.betfsm.Fallback].

This example also uses the GUI to visualize what is happening, and (in the comments) also
shows you how to debug by logging all entries/exits of BeTFSM nodes and displaying the active nodes.


This example can be found in the ```betfsm_examples/example_common_nodes.py``` file,
and is repeated below for your convenience:

``` py
#!/usr/bin/env python3

import time
from betfsm.betfsm import (
    TickingStateMachine, BeTFSMRunner, Sequence, ConcurrentSequence, 
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard,
    Repeat, Fallback, TimedWait, Concurrent, TimedRepeat
)
from betfsm.graphviz_visitor import to_graphviz_dotfile
from betfsm.logger import get_logger, LogPrinter, set_logger
from betfsm.betfsmrunnergui import BeTFSMRunnerGUI

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

class CancelingState(Generator):
    """
    A state that returns CANCEL after one tick.
    Used to demonstrate Fallback.
    """
    def __init__(self, name,count):
        super().__init__(name, [CANCEL])
        self.count = count

    def co_execute(self, blackboard):
        get_logger().info(f"{self.name}: I am going to cancel after {self.count} ticks...")
        for i in range(self.count): 
            yield TICKING
        get_logger().info(f"{self.name}: Canceling now!")
        yield CANCEL


"""
An example of a reusable adaptation:
"""
class CountDownWithDelay(TimedRepeat):
    def __init__(self,name,count, delay):
        self.msg = Message(msg=f"{name} counting down")
        super().__init__(name, count, delay, self.msg)


def main():

    # if you like to log all entry, doo and exit's of all the tickingstates:
    # set_logger("state", LogPrinter())

    # Create a blackboard
    bb = {}

    # 1. Example Sequence
    # This will execute its children one after another.
    seq = Sequence("sequence_phase", [
        Message(msg="--- Starting Sequence Phase ---"),
        CountDownWithDelay("seq_counter_1", 6, 1.0),
        Message(msg="--- Sequence Phase Finished ---")
    ])

    # 2. Example ConcurrentSequence 
    # This will execute its children concurrently. 
    # It succeeds when ALL children succeed.
    conc_seq = ConcurrentSequence("concurrentseq_phase", [
        Message(msg="--- Starting ConcurrentSequence Phase ---"),
        CountDownWithDelay("Concurrent_Counter_1", 5, 1.5),
        CountDownWithDelay("Concurrent_Counter_2", 3, 4.0),
    ])

    # 3. Example Concurrent
    # This will execute its children concurrently. 
    # It succeeds when one of the children succeeds.
    conc = Concurrent("concurrent_phase", [
        CountDownWithDelay("Concurrent_Counter_3", 5, 1.5),
        CountDownWithDelay("Concurrent_Counter_4", 3, 4.0),
    ])





    # 4. Example Repeat
    # Repeats the underlying state N times.
    rep = Repeat("repeat_phase", 3, Sequence("repseq",[Message(msg="--- Repeating Message (3 times) ---"), TimedWait("delay",1.0)]))

    # 5. Example Fallback : execute until one in the list succeeds.
    # Tries children in order. If one returns CANCEL, it tries the next.
    # If one returns anything else (e.g. SUCCEED), it returns that and stops.
    fallback = Fallback("fallback_phase", [
        CancelingState("Cancel_State_1",30),
        CancelingState("Cancel_State_2",20),
        Message(msg="Fallback recovered! (This message runs because previous states CANCELED) Since it returns SUCCEED, the whole fallback return SUCCEED.")
    ])

    # 6. State Machine Definition
    # Define the possible outcomes of the SM.
    #sm = TickingStateMachine("root_sm", [SUCCEED])
    
    # Add the states to the State Machine and link them
    #sm.add_state(seq, transitions={SUCCEED: conc_seq}) # you can refer to a state by its instance
    #sm.add_state(conc_seq, transitions={SUCCEED: conc})
    #sm.add_state(conc, transitions={SUCCEED: rep})
    #sm.add_state(rep, transitions={SUCCEED: "fallback_phase"}) # you can also refer to a state by its name
    #sm.add_state(fallback, transitions={SUCCEED: SUCCEED}) # If fallback succeeds (returns SUCCEED), SM finishes. 
                                                           # if the value in the transitions map is an outcome, it indicates that the statemachine exits. 
    # Set the initial state
    #sm.set_start_state("sequence_phase")

    sm = Sequence("application",[seq, conc_seq, conc, rep, fallback])
    
    to_graphviz_dotfile("example_runner_gui.dot", sm)

    # 7. Run it using BeTFSMRunner at 100 Hz
    # display_active == True if you'd like to log all active states
    runner = BeTFSMRunnerGUI(sm, bb, frequency=10.0,publish_frequency=5,display_active=False) # Hz
    
    print("Running State Machine...")
    outcome = runner.run()
    print(f"State Machine Finished with outcome: {outcome}")

if __name__ == "__main__":
    main()

```


