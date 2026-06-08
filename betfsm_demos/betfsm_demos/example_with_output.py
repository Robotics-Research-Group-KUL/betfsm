#!/usr/bin/env python3

#
# Example with output of crospi tasks.
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

import sys
import rclpy

from betfsm import (
    Sequence,  Repeat,
    CANCEL, NO_EVENT,
    set_logger,
    EventSequential, Ctrl_C_Condition, Concurrent
)
from betfsm_crospi import load_task_list, CrospiTask, CrospiDeactivate,CrospiOutput_v2
from betfsm_ros import BeTFSMNode,ROSRunner



class MySequence(Sequence):
    def __init__(self):
        super().__init__("my_sequence", [
            CrospiTask("MovingHome","MovingHome"),
            CrospiTask("MovingDown","MovingDown"),
            CrospiTask("MovingUp","MovingUp"),
            Concurrent("MovingSpline",[
                CrospiOutput("output","/my_topic",queue_size=20,path="../output"),
                CrospiTask("MovingSpline","MovingSpline")
            ])
        ] )

# main
def main(args=None): rclpy.init(args=args)    my_node = BeTFSMNode.get_instance("example_with_output")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    
    blackboard = {}
    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)    

    # running MySequence() and cleaning up when CTRL_C is pressed        
    nominal_sm = Repeat("repeat",-1,MySequence())
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
