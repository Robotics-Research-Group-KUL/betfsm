# ---- building the state machine -----
from betfsm import (
        Sequence, ConcurrentSequence, TimedWait,  Message, SUCCEED, Repeat, get_logger, BeTFSMRunnerGUI
)

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

def main():
    bb = {}
    sm = build_tree_larger()
    runner = BeTFSMRunnerGUI(sm,bb, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    runner.run()

if __name__ == "__main__":
    main()



