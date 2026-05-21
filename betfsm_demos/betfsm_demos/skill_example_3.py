#!/usr/bin/env python3

#
# using a Sequence (stops after one time going through the sequence)
#
# Also illustrates proper shutdown
#
# If control-C is pressed, a cleanup statemachine is called that directly shuts down
# the currently running crospi task
#


#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Erwin Aertbelien
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
    Sequence,  Message, 
    SUCCEED, TICKING, CANCEL, ABORT, TIMEOUT, NO_EVENT,
    TickingStateMachine,get_logger,set_logger,
    EventSequential, Ctrl_C_Condition
)
from betfsm_crospi import load_task_list, CrospiTask, CrospiDeactivate
from betfsm_ros import BeTFSMNode,ROSRunner,Node,Duration,LifeCycle,Transition


class MySequence(Sequence):
    def __init__(self):
        super().__init__("my_sequence", [
            CrospiTask("MovingHome","MovingHome"),
            CrospiTask("MovingDown","MovingDown"),
            CrospiTask("MovingUp","MovingUp"),
            CrospiTask("MovingSpline","MovingSpline") ]
        )

# main
def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())

    get_logger().info("skill_example_3 started")
    get_logger().info("This example cleans up after a ctrl-c using a cleanup state machine")
    
    blackboard = {}
    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    

    # running MySequence() and cleaning up when CTRL_C is pressed        
    nominal_sm = MySequence()
    cleanup_sm = CrospiDeactivate(force_outcome=CANCEL)
    sm = EventSequential("check_cancel", Ctrl_C_Condition("CTRL_C",repeated=3),{NO_EVENT:nominal_sm, "CTRL_C":cleanup_sm})

    # ROSRunner accepts command-line parameters (see --help)
    # has many more optional arguments than used below, see API documentation
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
