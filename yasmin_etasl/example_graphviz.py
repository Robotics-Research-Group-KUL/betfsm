#!/usr/bin/env python3

# example_graphviz.py
#
# Copyright (C) Erwin Aertbeliën,  2024
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


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

    rclpy.init(args=args)

    my_node = YasminTickingNode.get_instance("example_graphviz")
    set_logger(my_node)
    blackboard = Blackboard()

    load_task_list("$[yasmin_etasl]/tasks/my_tasks.json",blackboard)
    
  

    # # SOME BUG: 
    # sm = ConcurrentSequence("parallel", children=[
    #         ("task1", Sequence("my_sequence", children=[
    #                     ("movinghome",eTaSL_StateMachine("MovingHome") ),
    #                     ("movingup",eTaSL_StateMachine("MovingUp") ),
    #                     ("movingdown",eTaSL_StateMachine("MovingDown") ),            
    #                     ("movingup",eTaSL_StateMachine("MovingUp")),
    #                     ("my_message",Message("Hello world"))
    #                   ]) 
    #         ),
    #         ("task2",Sequence("timer", children=[
    #                     ("timer",TimedWait(Duration(seconds=3.0) ) ),
    #                     ("hello",Message("Timer went off!"))
    #                 ])
    #         )
    # ])
    sm=ConcurrentSequence("up_and_down_as_a_function", children=[
            ("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("movinghome","MovingHome") ),
                        ("movingup",eTaSL_StateMachine("movingup","MovingUp") ),
                        ("movingdown",eTaSL_StateMachine("movingdown","MovingDown") ),            
                        ("movingup",eTaSL_StateMachine("movingup2","MovingUp")),
                        ("my_message",Message("Robot is finished"))
                      ]) 
            ),
            ("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0) ) ),
                        ("hello",Message("Timer went off!"))
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
