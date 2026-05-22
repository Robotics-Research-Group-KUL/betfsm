#!/usr/bin/env python3

import time
from betfsm import (
    TICKING,SUCCEED,CANCEL,NO_EVENT,
    Runner, Sequence,  
    EventOutcome, EventSequential, EventConcurrent,
    Ctrl_C_Condition, Timeout_Condition,
    Message, Generator, get_logger, AlwaysOutcome
)

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

# The sequence defined as a class:

class MySequence(Sequence):
    def __init__(self, count):
        super().__init__("MySequence") # do not forget to initialize the super class
        self.count = count
        self.add_state( Message(msg="--- Starting Sequence Phase ---") )
        self.add_state( CountDown("seq_counter_1", self.count) )
        self.add_state( Message(msg="--- Sequence Phase Finished --- (message 1 of 2)"))
        self.add_state( Message(msg="--- Sequence Phase Finished --- (message 2 of 2)"))


def CleanupSeq():
    seq = Sequence("cleanup")
    seq.add_state(Message(msg="--Now cleaning up, takes 5 seconds --"))
    # as an alternative to TimedWait, we can use the event mechanism 
    # with a timer event, this blocks until DONE is received:
    waiting = EventOutcome("Waiting",Timeout_Condition("DONE",5.0),
                           event_map={"DONE":SUCCEED})
    seq.add_state(waiting)
    seq.add_state(Message(msg="---Done cleaning up ---"))
    # make sure that the sequence will generate outcome CANCEL, such that 
    # the nominal_sm is interrupted and stopped.
    seq.add_state(AlwaysOutcome(CANCEL))
    return seq

def main():
    # Create a blackboard
    bb = {}

    # 1. Example Sequence, defined as a custom class:
    nominal_sm = MySequence(100)
    cleanup_sm = CleanupSeq()


    # 2. nominal_sm will be paused when handling CTRL_C
    # This ends when nominal_sm returns with outcome != TICKING or
    # cleanup_sm returns with outcome !=TICKING or != SUCCEED
    sm = EventSequential("ctrl-c-check",Ctrl_C_Condition("CTRL_C"),{
        NO_EVENT:  nominal_sm,
        "CTRL_C" : cleanup_sm
    })

    # 2. Run it using BeTFSMRunner at 2 Hz
    runner = Runner(sm, bb, frequency=2) # Hz
    get_logger().info("Running State Machine... (explicitly logged in main)")
    outcome = runner.run()
    get_logger().info(f"State Machine Finished with outcome: {outcome} (explicitly logged in main)")
    

if __name__ == "__main__":
    main()

