#!/usr/bin/env python3

import time
from betfsm.betfsm import (
    TickingStateMachine, BeTFSMRunner, Sequence, ConcurrentSequence, 
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard,
    Repeat, Fallback
)
from betfsm.logger import get_logger

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
    def __init__(self, name):
        super().__init__(name, [CANCEL])

    def co_execute(self, blackboard):
        get_logger().info(f"{self.name}: I am going to cancel...")
        yield TICKING
        get_logger().info(f"{self.name}: Canceling now!")
        yield CANCEL

def main():
    # Create a blackboard
    bb = {}

    # 1. Example Sequence
    # This will execute its children one after another.
    seq = Sequence("sequence_phase", [
        Message(msg="--- Starting Sequence Phase ---"),
        CountDown("seq_counter_1", 4),
        Message(msg="--- Sequence Phase Finished ---")
    ])

    # 2. Example ConcurrentSequence
    # This will execute its children concurrently. 
    # It succeeds when ALL children succeed.
    conc_seq = ConcurrentSequence("concurrent_phase", [
        Message(msg="--- Starting Concurrent Phase ---"),
        CountDown("Concurrent_Counter_1", 5),
        CountDown("Concurrent_Counter_2", 3),
    ])

    # 3. Example Repeat
    # Repeats the underlying state N times.
    rep = Repeat("repeat_phase", 3, Message(msg="--- Repeating Message (3 times) ---"))

    # 4. Example Fallback
    # Tries children in order. If one returns CANCEL, it tries the next.
    # If one returns anything else (e.g. SUCCEED), it returns that and stops.
    fallback = Fallback("fallback_phase", [
        CancelingState("Cancel_State_1"),
        CancelingState("Cancel_State_2"),
        Message(msg="Fallback recovered! (This message runs because previous states CANCELED)")
    ])

    # 5. State Machine Definition
    # Define the possible outcomes of the SM.
    sm = TickingStateMachine("root_sm", [SUCCEED])
    
    # Add the states to the State Machine and link them
    sm.add_state(seq, transitions={SUCCEED: "concurrent_phase"})
    sm.add_state(conc_seq, transitions={SUCCEED: "repeat_phase"})
    sm.add_state(rep, transitions={SUCCEED: "fallback_phase"})
    sm.add_state(fallback, transitions={SUCCEED: SUCCEED}) # If fallback succeeds (returns SUCCEED), SM finishes.
    
    # Set the initial state
    sm.set_start_state("sequence_phase")

    # 6. Run it using BeTFSMRunner at 100 Hz
    runner = BeTFSMRunner(sm, bb, frequency=100.0) # Hz
    
    print("Running State Machine...")
    outcome = runner.run()
    print(f"State Machine Finished with outcome: {outcome}")

if __name__ == "__main__":
    main()
