# Example of a sequence
#!/usr/bin/env python3

import time
from betfsm.betfsm import (
    BeTFSMRunner, Sequence, ConcurrentSequence,TimedWait,TimedRepeat,
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
    
    sm = Sequence("concurrent_sequence_outer",[
            Message(msg="This demo uses ConcurrentSequence, Sequence, TimedRepeat"),
            Message(msg="--- concurrent_sequence started ---"),
            ConcurrentSequence("concurrent_sequence",[
                    Sequence("sequence1", [
                        Message(msg="   --- sequence 1 started ---"),
                        TimedRepeat("timedrepeat1",5, 1.5, Message(msg="      sequence 1: 5 times every 1.5 second")),
                        Message(msg="   --- sequence 1 ended   ---")]),
                    Sequence("sequence2", [
                        Message(msg="   --- sequence 2 started ---"),
                        TimedRepeat("timedrepeat1", 10, 0.6, Message(msg="      sequence 2: 10 times every 0.6 second")),
                        Message(msg="   --- sequence 2 ended   ---")]),
                    Sequence("sequence3", [TimedWait("waiting 5 sec",5.0), Message(msg="I like to interrupt!",logFunc=get_logger().warn) ]),
                ]),
            Message(msg="--- concurrent_sequence ended   ---")
        ])

    # 2. visualize in a dot file
    to_graphviz_dotfile("example_concurrentseq.dot", sm)

    # 3. Run it using BeTFSMRunner at 100 Hz

    # Here we show a demo of the debugging output of BeTFSMRunner, 
    runner = BeTFSMRunner(sm, bb, frequency=10.0, debug=True, display_active=True)
    get_logger().info("Running State Machine...")
    outcome = runner.run()
    get_logger().info(f"State Machine Finished with outcome: {outcome}")
    
if __name__ == "__main__":
    main()

