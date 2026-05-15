#!/usr/bin/env python3

#
# Using a TickingStateMachine ( infinite loop)
#
# Also illustrates proper shutdown
# 
# If control-C is pressed, the currently running crospi task will continue to run
#

#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui, Erwin Aertbelien
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

from betfsm import (
    SUCCEED, TICKING, CANCEL, TIMEOUT,ABORT, TickingStateMachine, get_logger,set_logger
)
from betfsm_crospi import load_task_list, eTaSL_StateMachine
from betfsm_ros import BeTFSMNode,ROSRunner


class MyStateMachine(TickingStateMachine):
    def __init__(self):
        super().__init__("my_state_machine",[SUCCEED, ABORT,TIMEOUT])

        # you can also use the names of the states in the transitions, but using the variables
        # avoids issues with spelling errors in the name and is "cleaner"
        movinghome   = eTaSL_StateMachine("MovingHome","MovingHome")
        movingdown   = eTaSL_StateMachine("MovingDown","MovingDown")
        movingup     = eTaSL_StateMachine("MovingUp","MovingUp")
        movingspline = eTaSL_StateMachine("MovingSpline","MovingSpline")

        self.add_state(movinghome, transitions={
            SUCCEED:   movingdown
        })
        self.add_state(movingdown,transitions={
            SUCCEED:   movingup
        })
        self.add_state(movingup, transitions={
            SUCCEED:   movingspline
        })
        self.add_state(movingspline, transitions={
            SUCCEED:   movinghome
        })


# main
def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    #set_logger("service",my_node.get_logger()) 
    #set_logger("state",my_node.get_logger())   


    get_logger().info("skill_example_1 started")
    blackboard = {}

    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    
    get_logger().info("Creating state machine: ")
    sm = MyStateMachine()

    runner = ROSRunner(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)

    try:
        runner.run()
    except KeyboardInterrupt:
        my_node.destroy_node()
        return   
    my_node.destroy_node()
    rclpy.shutdown()
    print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))
