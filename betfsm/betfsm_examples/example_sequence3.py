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

# The sequence defined as a class:

class MySequence(Sequence):
    def __init__(self, count):
        super().__init__("MySequence") # do not forget to initialize the super class
        self.count = count
        self.add_state( Message(msg="--- Starting Sequence Phase ---") )
        self.add_state( CountDown("seq_counter_1", self.count) )
        self.add_state( Message(msg="--- Sequence Phase Finished ---") )
        self.add_state( Message(msg="--- Sequence Phase Finished --- (message 1 of 2)"))
        self.add_state( Message(msg="--- Sequence Phase Finished --- (message 2 of 2)"))
 

def main():
    # Create a blackboard
    bb = {}

    # 1. Example Sequence
    # This time, we define the sequence as a class definition
    sm = MySequence(5)

    # 2. Run it using BeTFSMRunner at 100 Hz
    runner = BeTFSMRunner(sm, bb, frequency=100.0,debug=True) # Hz
    get_logger().info("Running State Machine... (explicitly logged in main)")
    outcome = runner.run()
    get_logger().info(f"State Machine Finished with outcome: {outcome} (explicitly logged in main)")
    

if __name__ == "__main__":
    main()

