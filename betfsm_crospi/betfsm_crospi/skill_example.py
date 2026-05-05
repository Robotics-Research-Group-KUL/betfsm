#!/usr/bin/env python3

#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui, Erwin Aertbelien
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

import rclpy
import sys


import time
from betfsm.betfsm import (
    BeTFSMRunner, Sequence,  
    Message, SUCCEED, TICKING, CANCEL, Generator, Blackboard,
    TickingState,TickingStateMachine
)
from betfsm.graphviz_visitor import to_graphviz_dotfile
from betfsm.logger import get_logger




#from threading import Lock
#import rclpy.time
#from rclpy.node import Node

# from betfsm.betfsm import TickingState,TICKING,Blackboard, Sequence, Message, TickingStateMachine
#from betfsm.betfsm import *
#from betfsm.betfsm_node import BeTFSMNode
from betfsm_crospi import load_task_list, eTaSL_StateMachine
from betfsm.graphviz_visitor import GraphViz_Visitor
from betfsm.logger import get_logger,set_logger
from betfsm_ros.betfsm_ros import *


from rclpy.duration import Duration


#from betfsm.graphviz_visitor import *


# from betfsm.betfsm_action_server import CheckForCanceledAction

import math

# from skill_requirements import ParamValidator as pv

# param = pv.parameters("Task description goes here", [
#     pv.p_scalar({"name": "maxvel", "description": "Maximum velocity [m/s]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "maxacc", "description": "Maximum acceleration [m/s^2]", "default": 0.1, "required": True, "maximum": 0.5}),
#     pv.p_scalar({"name": "eq_r", "description": "Equivalent radius", "default": 0.08, "required": False}),
#     pv.p_string({"name": "task_frame", "description": "Name of frame used to control the robot in cartesian space", "default": "tcp_frame", "required": False}),
#     pv.p_array({"name": "delta_pos", "type": "number", "default": [0.0, 0.0, 0.0], "description": "3D array of distances [m] that the robot will move w.r.t. the starting position in the X,Y,Z coordinates", "required": True, "minimum": -1.5, "maximum": 1.5, "minItems": 3, "maxItems": 3}),
# ])

#
#class BeTFSMRunner:
#    def __init__(self,node:Node, statemachine:TickingState, blackboard: Blackboard, sampletime):
#        get_logger().info(f"BeTFSMRunner started with sample time {sampletime}")
#        self.node  = node
#        self.sm    = statemachine
#        self.bm    = blackboard
#        self.timer = self.node.create_timer(sampletime, self.timer_cb)
#        self.outcome = "TICKING"
#        self.outcome_lock = Lock()
#
#    def timer_cb(self):
#        outcome = self.sm(self.bm)
#        #resetprint("---"),
#        if outcome!=TICKING:
#            self.timer.cancel()
#            self.set_outcome(outcome)
#            self.sm.reset()
#
#    def set_outcome(self, outcome):
#        with self.outcome_lock:
#            self.outcome = outcome
#
#    def get_outcome(self):
#        with self.outcome_lock:
#            outcome = self.outcome
#        return outcome
#


class MyStateMachine(TickingStateMachine):
    def __init__(self):
        super().__init__("my_state_machine",[SUCCEED, ABORT])

        # self.add_state(
        #     TimedWait("A",Duration(seconds=1.0)),             
        #     transitions={SUCCEED: "MovingDown", 
        #                 ABORT: ABORT})

        # Uncomment for admittance
        # self.add_state(
        #     eTaSL_StateMachine("MovingHome","MovingHome",node=None), 
        #     transitions={SUCCEED: "MovingAdmittance", 
        #                 ABORT: ABORT}
        # )

        # self.add_state(
        #     eTaSL_StateMachine("MovingAdmittance","MovingAdmittance",node=None), 
        #     transitions={SUCCEED: SUCCEED, 
        #                 ABORT: ABORT}
        # )

        # Uncomment for example sequence
        self.add_state(
            eTaSL_StateMachine("MovingHome","MovingHome",node=None), 
            transitions={SUCCEED: "MovingDown", 
                        ABORT: ABORT}
        )

        self.add_state(
            eTaSL_StateMachine("MovingDown","MovingDown",node=None), 
            transitions={SUCCEED: "MovingUp", 
                        ABORT: ABORT}
        )

        self.add_state(
            eTaSL_StateMachine("MovingUp","MovingUp",node=None), 
            transitions={SUCCEED: "MovingSpline", 
                        ABORT: ABORT}
        )

        self.add_state(
            eTaSL_StateMachine("MovingSpline","MovingSpline",node=None), 
            transitions={SUCCEED: "MovingHome", 
                        ABORT: ABORT}
        )

        # Uncomment for example sequence
        # self.add_state(
        #     eTaSL_StateMachine("MovingHome","MovingHome",node=None), 
        #     transitions={SUCCEED: "keep_frame", 
        #                 ABORT: ABORT}
        # )

        # self.add_state(
        #     eTaSL_StateMachine("keep_frame","keep_frame",node=None), 
        #     transitions={SUCCEED: SUCCEED, 
        #                 ABORT: ABORT}
        # )

        # self.add_state(
        #     eTaSL_StateMachine("follow_position_vector","follow_position_vector",node=None), 
        #     transitions={SUCCEED: SUCCEED, 
        #                 ABORT: ABORT}
        # )
        # self.add_state(
        #     eTaSL_StateMachine("follow_tf","follow_tf",node=None), 
        #     transitions={SUCCEED: SUCCEED, 
        #                 ABORT: ABORT}
        # )

    #         sm_out.add_state("MovingHome", etasl_utils.nested_etasl_state(name="MovingHome",  display_in_viewer=True),
    #                 transitions={SUCCEED: "MovingDown", 
    #                              ABORT: ABORT})

    # sm_out.add_state("MovingDown", etasl_utils.nested_etasl_state(name="MovingDown",  display_in_viewer=True),
    #                 transitions={SUCCEED: "MovingUp", 
    #                              ABORT: ABORT})
    
    # sm_out.add_state("MovingUp", etasl_utils.nested_etasl_state(name="MovingUp",  display_in_viewer=True),
    #             transitions={SUCCEED: "MovingDown", 
    #                              ABORT: ABORT})

        # self.add_state(
        #     Sequence("B", [
        #         TimedWait("a",Duration(seconds=1.0)), 
        #         TimedWait("b",Duration(seconds=1.0)), 
        #         TimedWait("c",Duration(seconds=1.0))
        #     ]), 
        #     transitions={SUCCEED:"C"}
        # )

        # self.add_state(TimedWait("C",Duration(seconds=1.0)), transitions={SUCCEED:"A"})


# main
def main(args=None):

    print("betfsm")
    rclpy.init(args=args)

    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())

    blackboard = {}

    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    
    sm = MyStateMachine()

    # prints a graphviz representation of sm:
    # vis = GraphViz_Visitor()
    # sm.accept(vis)
    # vis.print()

    #def __init__(self, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, debug: bool = False, display_active=False):
    #runner = BeTFSMRunner(my_node,sm,blackboard,0.01)

    # ERWIN: HANGS FOR NOW??  
    runner = BeTFSMROSRunner(my_node,sm,blackboard,0.01)

    rclpy.spin(my_node)
    
    # try:
    #     while (runner.get_outcome()==TICKING):
    #         rclpy.spin_once(my_node)
    #     rclpy.shutdown()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     print("final outcome : ",runner.get_outcome())
        
    print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))
