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
from .yasmin_ticking import *
from .yasmin_ticking_ros import *
from .yasmin_ticking_etasl import *
from .graphviz_visitor import *

from . import sm_up_and_down
from .logger import get_logger,set_logger



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

# main
def main(args=None):

    print("yasmin_etasl")
    rclpy.init(args=args)

    my_node = YasminNode.get_instance()

    set_logger("default",my_node.get_logger())
    #set_logger("service",my_node.get_logger())
    #set_logger("state",my_node.get_logger())



    blackboard = Blackboard()

    load_task_list("$[yasmin_etasl]/tasks/my_tasks.json",blackboard)
    
 
    sm = sm_up_and_down.Up_and_down_as_a_class()

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
