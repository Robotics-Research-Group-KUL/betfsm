import threading
import webbrowser

# ---- building the state machine -----
from betfsm.betfsm import (
        Sequence, ConcurrentSequence, TimedWait, TimedRepeat, Message, SUCCEED, Generator, Repeat
)
from betfsm.logger import get_logger
from betfsm.betfsmrunnergui import BeTFSMRunnerGUI

def build_tree():
    sm = Sequence("concurrent_sequence_outer", [
        Message(msg="This demo uses ConcurrentSequence, Sequence, TimedRepeat"),
        Message(msg="--- concurrent_sequence started ---"),
        ConcurrentSequence("concurrent_sequence", [
            Sequence("sequence1", [
                Message(msg="   --- sequence 1 started ---"),
                TimedRepeat("timedrepeat1", 5, 5, Message(msg="      sequence 1: 5 times every 5 second")),
                Message(msg="   --- sequence 1 ended   ---")]),
            Sequence("sequence2", [
                Message(msg="   --- sequence 2 started ---"),
                TimedRepeat("timedrepeat1", 10, 6, Message(msg="      sequence 2: 10 times every 6 second")),
                Message(msg="   --- sequence 2 ended   ---")]),
            Sequence("sequence3", [
                TimedWait("waiting 20 sec", 20.0),
                Message(msg="I like to interrupt!", logFunc=get_logger().warn)]),
        ]),
        Message(msg="--- concurrent_sequence ended   ---")
    ])
    return sm

def subtree(nr:int, sz:int, sec: float):
    sm = Sequence(f"subseq{nr}")
    for i in range(sz):
        sm.add_state( TimedWait("waiting {sec}",sec)),
        sm.add_state( Message(msg=f"subseq{nr} says {i}"))
    return sm

def build_tree2():
    return Repeat("repeat",-1,Sequence("concurrent_sequence_outer",[
            Message(msg="This demo uses ConcurrentSequence, Sequence, TimedRepeat"),
            Message(msg="--- concurrent_sequence started ---"),
            ConcurrentSequence("concurrent_sequence",[
                    Sequence("sequence1", [
                        Message(msg="   --- sequence 1 started ---"),
                        TimedRepeat("timedrepeat1",5, 1.5, Message(msg="      sequence 1: 5 times every 1.5 second")),
                        Message(msg="   --- sequence 1 ended   ---")]),
                    Sequence("sequence2", [
                        Message(msg="   --- sequence 2 started ---"),
                        TimedRepeat("timedrepeat1", 10, 0.6, Message(msg="      sequence 2: 10 times every 0.4 second")),
                        Message(msg="   --- sequence 2 ended   ---")]),
                    Sequence("sequence3", [TimedWait("waiting 3 sec",5.0), Message(msg="I like to interrupt!",logFunc=get_logger().warn) ]),
                ]),
            Message(msg="--- concurrent_sequence ended   ---")
        ])
    )

 
def build_tree_larger():
    sm = Repeat("repeat_forever",-1,
                Sequence("seq1",[
                    Message(msg="sequence seq1 started"),
                    ConcurrentSequence("concurrent_seq",[
                        subtree(1,10,2.0),
                        subtree(2,10,3.0)
                        ]),
                    Message(msg="sequence seq1 ended"),
                    Message(msg="sequence seq2 started"),
                    subtree(3,5,1.0),
                    Message(msg="sequence seq2 ended")
                ])
          )
    return sm



# ---------------------------------------


def run_machine():
    bb = {}
    sm = build_tree2() #_larger()
    runner = BeTFSMRunnerGUI(sm, bb, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    runner.run()

if __name__ == "__main__":
    # Start the runner in a background thread
    threading.Thread(target=run_machine, daemon=True).start()

    # Start FastAPI server
    import uvicorn
    # webbrowser.open("http://127.0.0.1:8000/static/index.html")
    uvicorn.run("betfsm.backend.app:app", host="0.0.0.0", port=8000, reload=False)

