#!/usr/bin/env python3

#
# using a Sequence (stops after one time going through the sequence)
#
# Also illustrates proper shutdown
#
# If control-C is pressed, the loop continues until a node that checks the result and
# the application is stopped at that location
#
# Note: the same CheckCancel is used as in skill_example_3 in a totally different way,
# demonstrating the flexibility of the mechanism.


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
    Sequence,  Repeat, Message, AlwaysOutcome,
    SUCCEED, TICKING, CANCEL, ABORT,TIMEOUT,NO_EVENT,
    get_logger,set_logger,
    EventOutcome, Ctrl_C_Condition
)
from betfsm_crospi import load_task_list, CrospiTask
from betfsm_ros import BeTFSMNode, RunnerBase,ROSRunner


class MyTree(Repeat):
    def __init__(self):
        sequence = Sequence("my_sequence", [
            Message(None,msg="start of a new loop, ctrl-c will interrupt the cylce only directly after this message"),
            EventOutcome("check_ctrl_c",Ctrl_C_Condition(), {"CTRL_C":CANCEL, NO_EVENT:SUCCEED}),
            CrospiTask("MovingHome","MovingHome"),
            CrospiTask("MovingDown","MovingDown"),
            CrospiTask("MovingUp","MovingUp"),
            CrospiTask("MovingSpline","MovingSpline")
        ])
        super().__init__("my_tree",-1,sequence)


def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())

    get_logger().info("skill_example_4 started")
    blackboard = {}

    load_task_list("$[betfsm_demos]/tasks/skill_example.json",blackboard)
    
    get_logger().info("Now cleaning up after a ctrl-c using a cleanup TickingState")


    
    sm = MyTree()
    

    # This is now working and recommended, accepts command-line parameters (see --help)
    # has many more optional arguments, see API documentation
    # checks whether timing exceeds sample period.
    #runner = BeTFSMRosRunnerGUI(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
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
