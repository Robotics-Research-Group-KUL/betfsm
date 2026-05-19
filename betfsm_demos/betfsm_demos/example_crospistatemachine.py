#!/usr/bin/env python3

#
# Using a TickingStateMachine ( infinite loop)
#
# Also illustrates proper shutdown of node
# 
# If control-C is pressed, the currently running crospi task will continue to run
#

import rclpy
import sys

from betfsm import (
    SUCCEED, TICKING, CANCEL, TIMEOUT,ABORT,NO_EVENT,
    get_logger,set_logger,Sequence, Repeat,
    EventSequential, ctrl_c_polling_func
)

from betfsm_ros import (
    BeTFSMNode,ROSRunner
)
                        
from betfsm_crospi import (
    load_task_list
)

import crospistatemachine as csm



def my_task():
    return Repeat("My_Task",-1, Sequence("cycle",[
        csm.CrospiTask("movingHome","MovingHome"),
        csm.CrospiTask("MovingDown","MovingDown"),
        csm.CrospiTask("MovingUp","MovingUp"),
        csm.CrospiTask("MovingSpline","MovingSpline")
    ]))




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
    
    sm = EventSequential("ctrl_c_check", ctrl_c_polling_func(),{
        NO_EVENT: my_task(),
        "CTRL_C": csm.CrospiDeactivate(force_outcome="CANCEL")
    })


    runner = ROSRunner(
        my_node,sm,blackboard, 
        frequency=100.0, 
        publish_frequency=5.0, 
        debug=False, 
        display_active=False)

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
