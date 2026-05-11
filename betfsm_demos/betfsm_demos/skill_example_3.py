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
    Sequence,  Message, SUCCEED, TICKING, CANCEL, ABORT,TIMEOUT,Generator, Blackboard,
    TickingState,TickingStateMachine,BeTFSMRunnerGUI, get_logger,set_logger,
    Ctrl_C_Handler,CheckCancel, get_path_value
)
from betfsm_crospi import load_task_list, eTaSL_StateMachine
from betfsm_ros import BeTFSMNode,ROSRunner,Node,Duration,LifeCycle,Transition


class MySequence(Sequence):
    def __init__(self):
        super().__init__("my_sequence", [
            eTaSL_StateMachine("MovingHome","MovingHome"),
            eTaSL_StateMachine("MovingDown","MovingDown"),
            eTaSL_StateMachine("MovingUp","MovingUp"),
            eTaSL_StateMachine("MovingSpline","MovingSpline") ]
        )

class MyCleanup(TickingStateMachine):
    """
    Cleaning up eTaSL i.e. stop any motion and put the crospi node in a cleaned up state
    """
    def __init__(self,srv_name:str="/crospi_node",timeout:Duration = Duration(seconds=0.1), node : Node = None):
        # execute in sequence but don't care about ABORT, only way to fail is TIMEOUT
        super().__init__("Cleanup",[CANCEL,SUCCEED])

        msg        =  Message(msg="State machine to cleanup is now running")
        deactivate =  LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node)
        cleanup    =  LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node)

        self.add_state(msg, transitions={
            SUCCEED:deactivate
        })
        self.add_state(deactivate, transitions={
            SUCCEED: cleanup,
            ABORT:   cleanup,
            TIMEOUT: cleanup
        })
        self.add_state(cleanup, transitions={
            SUCCEED:     CANCEL,
            TIMEOUT:     CANCEL,
            ABORT:       CANCEL
        })


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
    
    
    Ctrl_C_Handler(blackboard,"/cancelation/ctrl_c",repeated=3)    
    nominal_sm = MySequence()
    cleanup_sm = MyCleanup(node=my_node)
    sm = CheckCancel("check_cancelation", lambda bb: get_path_value(bb,"/cancelation/ctrl_c"), nominal_sm, cleanup_sm)

    # This is now working and recommended, accepts command-line parameters (see --help)
    # has many more optional arguments, see API documentation
    # checks whether timing exceeds sample period.
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
