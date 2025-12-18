import threading
import webbrowser

# ---- building the state machine -----
from betfsm.betfsm import (
        Sequence, ConcurrentSequence, TimedWait, TimedRepeat, Message, SUCCEED, Generator, Repeat
)
from betfsm.logger import get_logger
from betfsm.betfsmrunnergui import BeTFSMRunnerGUI

# ---------------------------------------
def subtree(nr:int, sz:int, sec: float):
    sm = Sequence(f"subseq{nr}")
    for i in range(sz):
        sm.add_state( TimedWait("waiting {sec}",sec)),
        sm.add_state( Message(msg=f"subseq{nr} says {i}"))
    return sm

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
    sm = build_tree_larger()
    runner = BeTFSMRunnerGUI(sm, bb, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    runner.run()

def main():
    # Start the runner in a background thread
    threading.Thread(target=run_machine, daemon=True).start()

    # Start FastAPI server
    import uvicorn
    # webbrowser.open("http://127.0.0.1:8000/static/index.html")
    uvicorn.run("betfsm.backend.app:app", host="0.0.0.0", port=8000, reload=False)

if __name__ == "__main__":
    main()

