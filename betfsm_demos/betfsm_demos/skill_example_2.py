#!/usr/bin/env python3

#
# using a Sequence (stops after one time going through the sequence)
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
    Sequence,  Message, SUCCEED, TICKING, CANCEL, ABORT,Generator, Blackboard,
    TickingState,TickingStateMachine,BeTFSMRunnerGUI, get_logger,set_logger
)
from betfsm_crospi import load_task_list, eTaSL_StateMachine
from betfsm_ros import BeTFSMNode,ROSRunner


class MySequence(Sequence):
    def __init__(self):
        super().__init__("my_sequence", [
            eTaSL_StateMachine("MovingHome","MovingHome"),
            eTaSL_StateMachine("MovingDown","MovingDown"),
            eTaSL_StateMachine("MovingUp","MovingUp"),
            eTaSL_StateMachine("MovingSpline","MovingSpline") ]
        )



# main
def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())

    get_logger().info("skill_example_2 started")
    blackboard = {}

    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    
    get_logger().info("Now using a sequence instead of a state machine to achieve the same thing...")
    sm = MySequence()

    # This is now working and recommended, accepts command-line parameters (call this file with --help argument)
    # has many more optional arguments, see API documentation
    # checks whether timing exceeds sample period.
    runner = ROSRunner(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=True, display_active=True)

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
