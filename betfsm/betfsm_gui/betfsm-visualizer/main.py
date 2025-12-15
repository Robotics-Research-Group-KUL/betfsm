import threading
from backend.app import BeTFSMRunnerGUI
import webbrowser

# ---- building the state machine -----
from betfsm.betfsm import (
        Sequence, ConcurrentSequence, TimedWait, TimedRepeat, Message, SUCCEED, Generator
)
from betfsm.logger import get_logger

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

root = build_tree()

# ---------------------------------------


def run_machine():
    bb = {}
    runner = BeTFSMRunnerGUI(root, bb, frequency=1.0, debug=True, display_active=True)
    runner.run()

if __name__ == "__main__":
    # Start the runner in a background thread
    threading.Thread(target=run_machine, daemon=True).start()

    # Start FastAPI server
    import uvicorn
    # webbrowser.open("http://127.0.0.1:8000/static/index.html")
    uvicorn.run("backend.app:app", host="0.0.0.0", port=8000, reload=False)

