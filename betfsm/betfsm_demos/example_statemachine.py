#!/usr/bin/env python3

#
# Example of a state machine
#

import time
from betfsm.betfsm import (
    BeTFSMRunner, Sequence,  TickingStateMachine,
    TimedWait, Message, TimedRepeat,
    SUCCEED, TICKING, CANCEL, Generator, Blackboard
)
from betfsm.graphviz_visitor import to_graphviz_dotfile
from betfsm.logger import get_logger
import random

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

# A user defined TickingState:
class FailSometimes(Generator):
    """
    Fails randomly with probability 1/n.
    """
    def __init__(self, name, n=3):
        super().__init__(name, [CANCEL,SUCCEED])
        self.n = n

    def co_execute(self, blackboard):
        get_logger().info(f"{self.name}: I will failing randomly 1 out of {self.n} times ")
        if random.randrange(self.n)==0:
            get_logger().info(f"{self.name}: CANCEL ")
            yield CANCEL
        else:
            get_logger().info(f"{self.name}: SUCCEED ")
            yield SUCCEED




class MyStateMachine(TickingStateMachine):
    def __init__(self,name):
        super().__init__(name,[SUCCEED])
        self.home = Message(msg="I am in the home state")
        self.counter = CountDown("counter", 3)
        self.waiting = TimedWait("waiting 1 second",1.0)
        self.failingstate = FailSometimes("checkFailure",3)
        self.final   = Message(msg="I am in the end state")

        self.add_state(self.home,transitions={SUCCEED:self.counter}) 
        self.add_state(self.counter,transitions={SUCCEED:self.waiting})
        self.add_state(self.waiting,transitions={SUCCEED:self.failingstate})
        self.add_state(self.failingstate,transitions={SUCCEED:self.counter,CANCEL:self.final})
        self.add_state(self.final,transitions={SUCCEED:SUCCEED})
        self.set_start_state(self.home)




def main():
    # Create a blackboard
    bb = {}

    # 1. Example Sequence
    # This will execute its children one after another.

    sm = MyStateMachine("my_state_machine")

    # 2. visualize in a dot file
    to_graphviz_dotfile("example_statemachine.dot", sm)

    # 3. Run it using BeTFSMRunner at 100 Hz
    runner = BeTFSMRunner(sm, bb, frequency=100.0) # Hz
    get_logger().info("Running State Machine...")
    outcome = runner.run()
    get_logger().info(f"State Machine Finished with outcome: {outcome}")
    
if __name__ == "__main__":
    main()

