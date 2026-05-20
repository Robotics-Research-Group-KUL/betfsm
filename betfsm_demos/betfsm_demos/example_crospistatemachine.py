#!/usr/bin/env python3

#
# This example brings together the new CrospiTask and
# some more complex event handling to deal with:
#   - ctrl-c : interrupt and cleanup
#   - STOP from HTTP : interrupt and cleanup
#   - OPEN, CLOSE from HTTP:  they should not interfere with each other, but can run concurrently with robot.
#   - PLAYSOUND from HTTP: can run concurrently with all of the above.
#

import rclpy
import sys

from betfsm import (
    SUCCEED, TICKING, CANCEL, TIMEOUT,ABORT,NO_EVENT,
    get_logger,set_logger,Sequence, Repeat, Message, TimedWait,
    EventSequential, ctrl_c_polling_func, http_polling_func,combine,EventConcurrent, ConcurrentSequence
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


def open_gripper():
    return Sequence("open_gripper",[
        Message(msg="Gripper starts to open (3 sec)"),
        TimedWait("waiting",3.0),
        Message(msg="Gripper finished opening")
    ])

def close_gripper():
    return Sequence("open_gripper",[
        Message(msg="Gripper starts to close (3 sec)"),
        TimedWait("waiting",3.0),
        Message(msg="Gripper finished closing")
    ])

def play_sound():
    return Sequence("play_sound",[
        Message(msg="Sound start playing for 5 sec"),
        TimedWait("waiting",5.0),
        Message(msg="Sound is stopped")
    ])

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

    nominal_sm = ConcurrentSequence("concurrentseq", [
                        my_task(),
                        EventSequential("sequential", http_polling_func(), {
                                 "OPEN" : open_gripper(),  "CLOSE" : open_gripper(),
                            }),
                        EventSequential("check_play_sound", http_polling_func(), {
                                 "PLAYSOUND": play_sound()
                                 })                            
                    ])
        

    # can't use twice the same subtree!
    sm = EventSequential("ctrl_c_check", combine( ctrl_c_polling_func(),http_polling_func()),{
        NO_EVENT: nominal_sm,
        "CTRL_C": csm.CrospiDeactivate(force_outcome="CANCEL"),
        "STOP": csm.CrospiDeactivate(force_outcome="CANCEL")
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
