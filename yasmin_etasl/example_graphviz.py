#!/usr/bin/env python3


import rclpy
import threading
import sys
#import rclpy.clock
from rclpy.duration import Duration
import rclpy.time
from yasmin_ros.yasmin_node import YasminNode
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub
from yasmin_action_server.yasmin_action_server import YasminActionServer,get_logger,set_logger, action_state_cb, CancelState,EmptyStateMachine
from yasmin_viewer import YasminViewerPub
from .yasmin_ticking import *
from .yasmin_ticking_ros import *
from .yasmin_ticking_etasl import *
from .graphviz_visitor import *



class MyMessage(Generator):
    """
    Message(msg) returns a State that displays a message
    """
    def __init__(self,msg) -> None:
        super().__init__("message",[SUCCEED,])
        self.msg = msg
    def co_execute(self,blackboard: Blackboard)-> str:
        my_node = YasminNode.get_instance()
        log = my_node.get_logger()
        log.info(f'Entering MyMessage : {self.msg}')
        yield SUCCEED




class YasminRunner:
    def __init__(self,node:Node, statemachine:TickingState, blackboard: Blackboard, sampletime):
        self.node  = node
        self.sm    = statemachine
        self.bm    = blackboard
        self.timer = self.node.create_timer(sampletime, self.timer_cb)
        self.outcome = "TICKING"
        self.outcome_lock = Lock()

    def timer_cb(self):
        outcome = self.sm(self.bm)
        #resetprint("---"),
        if outcome!=TICKING:
            self.timer.cancel()
            self.set_outcome(outcome)

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    my_node = YasminNode.get_instance()
    set_logger(my_node)
    blackboard = Blackboard()

    load_task_list("$[yasmin_etasl]/tasks/my_tasks.json",blackboard)
    
  

    # SOME BUG: 
    sm = ConcurrentSequence("parallel", children=[
            ("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("MovingHome") ),
                        ("movingup",eTaSL_StateMachine("MovingUp") ),
                        ("movingdown",eTaSL_StateMachine("MovingDown") ),            
                        ("movingup",eTaSL_StateMachine("MovingUp")),
                        ("my_message",MyMessage("Hello world"))
                      ]) 
            ),
            ("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0) ) ),
                        ("hello",MyMessage("Timer went off!"))
                    ])
            )
    ])



    vis = GraphViz_Visitor()
    sm.accept(vis)
    vis.print()
    rclpy.shutdown()
    return


if __name__ == "__main__":
    sys.exit(main(sys.argv))
