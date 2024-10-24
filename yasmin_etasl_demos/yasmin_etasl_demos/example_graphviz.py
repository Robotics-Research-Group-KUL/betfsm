# #!/usr/bin/env python3

# # example_graphviz.py
# #
# # Copyright (C) Erwin Aertbeliën,  2024
# #
# # This program is free software; you can redistribute it and/or
# # modify it under the terms of the GNU Lesser General Public
# # License as published by the Free Software Foundation; either
# # version 3 of the License, or (at your option) any later version.

# # This program is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# # Lesser General Public License for more details.

# # You should have received a copy of the GNU Lesser General Public License
# # along with this program; if not, write to the Free Software Foundation,
# # Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


# import rclpy
# import threading
# import sys
# #import rclpy.clock
# from rclpy.duration import Duration
# import rclpy.time
# from yasmin_ros.yasmin_node import YasminNode
# from yasmin import Blackboard, State
# from yasmin_ros.basic_outcomes import SUCCEED, ABORT
# from yasmin_viewer import YasminViewerPub
# from yasmin_action_server.yasmin_action_server import YasminActionServer,get_logger,set_logger, action_state_cb, CancelState,EmptyStateMachine
# from yasmin_viewer import YasminViewerPub
# from .yasmin_ticking import *
# from .yasmin_ticking_ros import *
# from .yasmin_ticking_etasl import *
# from .graphviz_visitor import *

# from . import sm_up_and_down as sud


# class YasminRunner:
#     def __init__(self,node:Node, statemachine:TickingState, blackboard: Blackboard, sampletime):
#         self.node  = node
#         self.sm    = statemachine
#         self.bm    = blackboard
#         self.timer = self.node.create_timer(sampletime, self.timer_cb)
#         self.outcome = "TICKING"
#         self.outcome_lock = Lock()

#     def timer_cb(self):
#         outcome = self.sm(self.bm)
#         #resetprint("---"),
#         if outcome!=TICKING:
#             self.timer.cancel()
#             self.set_outcome(outcome)

#     def set_outcome(self, outcome):
#         with self.outcome_lock:
#             self.outcome = outcome

#     def get_outcome(self):
#         with self.outcome_lock:
#             outcome = self.outcome
#         return outcome







# # main
# def main(args=None):

#     rclpy.init(args=args)

#     my_node = YasminTickingNode.get_instance("example_graphviz")
#     set_logger(my_node)
#     blackboard = Blackboard()

#     load_task_list("$[yasmin_etasl]/tasks/my_tasks.json",blackboard)
    
#     sm = sud.Up_and_down_with_parameters(my_node)



#     # # SOME BUG: 
#     # sm = ConcurrentSequence("parallel", children=[
#     #         ("task1", Sequence("my_sequence", children=[
#     #                     ("movinghome",eTaSL_StateMachine("MovingHome") ),
#     #                     ("movingup",eTaSL_StateMachine("MovingUp") ),
#     #                     ("movingdown",eTaSL_StateMachine("MovingDown") ),            
#     #                     ("movingup",eTaSL_StateMachine("MovingUp")),
#     #                     ("my_message",Message("Hello world"))
#     #                   ]) 
#     #         ),
#     #         ("task2",Sequence("timer", children=[
#     #                     ("timer",TimedWait(Duration(seconds=3.0) ) ),
#     #                     ("hello",Message("Timer went off!"))
#     #                 ])
#     #         )
#     # ])
    
#     vis = GraphViz_Visitor()
#     sm.accept(vis)
#     vis.print()
#     rclpy.shutdown()
#     return


# if __name__ == "__main__":
#     sys.exit(main(sys.argv))




#!/usr/bin/env python3

# test_ea.py
#
# Copyright (C) Erwin Aertbeliën, Santiago Iregui, 2024
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
import sys

import rclpy.time
from yasmin_ros.yasmin_node import YasminNode
from yasmin import Blackboard
# from yasmin_ros.basic_outcomes import SUCCEED, ABORT
# from yasmin_viewer import YasminViewerPub
from yasmin_etasl.yasmin_ticking import *
from yasmin_etasl.yasmin_ticking_ros import *
from yasmin_etasl.yasmin_ticking_etasl import *
from yasmin_etasl.graphviz_visitor import *

from . import sm_up_and_down
from yasmin_etasl.logger import get_logger,set_logger



class YasminRunner:
    def __init__(self,node:Node, statemachine:TickingState, blackboard: Blackboard, sampletime):
        get_logger().info(f"YasminRunner started with sample time {sampletime}")
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




class MyStateMachine(TickingStateMachine):
    def __init__(self):
        super().__init__("my_state_machine",[SUCCEED])

        self.add_state(TimedWait("A",Duration(seconds=1.0)), transitions={SUCCEED:"B"})

        self.add_state(
            Sequence("B", [
                TimedWait("a",Duration(seconds=1.0)), 
                TimedWait("b",Duration(seconds=1.0)), 
                TimedWait("c",Duration(seconds=1.0))
            ]), 
            transitions={SUCCEED:"C"}
        )

        self.add_state(TimedWait("C",Duration(seconds=1.0)), transitions={SUCCEED:"A"})



class PrintGraphviz(Generator):
    "To be able to print the graphviz while the state machine is running"
    def __init__(self, sm):
        super().__init__("print_graphviz",[SUCCEED])
        self.sm = sm

    def co_execute(self,bb):
        # prints a graphviz representation of sm:

        vis = GraphViz_Visitor()
        self.sm.accept(vis)
        vis.print()

        #vis = VisitorFullName()
        #self.sm.accept(vis)
        yield SUCCEED




def example():
    return ConcurrentSequence("concurrent",[ 
        Sequence("task1",[ 
            TimedWait("waiting",Duration(seconds=6.0)),
            Message(msg="finished waiting")
        ]),
        MyStateMachine()
    ])

# def add_graphviz_print( sm, period ):
#     return ConcurrentSequence("",[
#         TimedRepeat("repeat_every",10000,period,PrintGraphviz(sm)),
#         sm
#     ])

def run_for_specified_duration_while_publishing( sm, duration):
    return ConcurrentFallback("check_duration",[
        TimedWait("timer",duration),
        sm,
        GraphvizPublisher("publisher","/gz",sm,None,skip=25)
    ])




# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    my_node = YasminTickingNode.get_instance("test_ea")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    set_logger("state",my_node.get_logger())



    blackboard = Blackboard()

    load_task_list("$[yasmin_etasl]/tasks/my_tasks.json",blackboard)
    
 
    sm = run_for_specified_duration_while_publishing( example(), Duration(seconds=300))
    # sm.reset()
    # print(sm(blackboard))
    # prints a graphviz representation of sm:
    vis = GraphViz_Visitor()
    sm.accept(vis)
    vis.print()


    # YasminViewerPub("Complete FSM", sm)
    runner = YasminRunner(my_node,sm,blackboard,0.01)
    
    try:
        while (runner.get_outcome()=="TICKING"):
            rclpy.spin_once(my_node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        print("final outcome : ",runner.get_outcome())
        
        print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))
