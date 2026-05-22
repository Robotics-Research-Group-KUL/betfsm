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
    EventSequential, EventConcurrent, EventOutcome, ConcurrentSequence, AlwaysOutcome,
    TickingStateMachine, Callback_Condition,
    HTTPEvent_Condition,Ctrl_C_Condition,Timeout_Condition
)

from betfsm_ros import  BeTFSMNode,ROSRunner
from betfsm_crospi import  load_task_list, CrospiTask, CrospiDeactivate

import random

def my_task():
    return Repeat("My_Task",-1, Sequence("cycle",[
                        CrospiTask("MovingHome","MovingHome"),
                        CrospiTask("MovingDown","MovingDown"),
                        CrospiTask("MovingUp","MovingUp"),
                        CrospiTask("MovingSpline","MovingSpline")
                    ]))
    return 






def play_sound():
    return Sequence("play_sound",[
        Message(msg="Sound start playing for 5 sec"),
        TimedWait("waiting",5.0),
        Message(msg="Sound is stopped")
    ])

def complain():
    return Message(msg="Robots always need to work, can't I take a rest? My gears hurt.")


def quitting():
    return Sequence(None,[
        Message(msg="You didn't listen, I am done.  Do the work yourself!"),
        AlwaysOutcome(CANCEL)
    ])
                        
def open_gripper_simulation():
    return Sequence("opening_gripper",[
        Message(msg="start to open"),
        TimedWait("waiting",3.0),
        EventOutcome("check_random_failure",
                      Callback_Condition("FAIL",lambda _: random.random() < 0.3),
                      event_map={NO_EVENT: SUCCEED, "FAIL":CANCEL}
                    ),
        Message(msg="finished opening")
    ])
    
def close_gripper_simulation():
    return Sequence("closing_gripper",[
        Message(msg="start to close"),
        TimedWait("waiting",3.0),
        EventOutcome("check_random_failure",
                      Callback_Condition("FAIL",lambda _: random.random() < 0.3),
                      event_map={NO_EVENT: SUCCEED, "FAIL":CANCEL}
                    ),        
        Message(msg="finished closing")
    ])

def my_gripper_statemachine():
    sm = TickingStateMachine("gripper", [SUCCEED,CANCEL])
    
    opengripper  = open_gripper_simulation()
    closegripper = close_gripper_simulation()
    is_opened = EventOutcome("check_for_close",HTTPEvent_Condition(max_age=10),{    # listen to some older events
        "CLOSE": "close_outcome"
    })
    is_closed = EventOutcome("check_for_open",HTTPEvent_Condition(max_age=10),{     # listen to some older events
        "OPEN": "open_outcome"
    })

    sm.add_state(is_opened,   { "close_outcome": closegripper })
    sm.add_state(closegripper,{  SUCCEED:        is_closed    , CANCEL: opengripper  }) # if failure, open it again
    sm.add_state(is_closed,   {  "open_outcome": opengripper  })
    sm.add_state(opengripper, {  SUCCEED:        is_opened    , CANCEL: closegripper }) # if failure, close it again

    return sm
    



# main
def main(args=None):
    doc = """
    Reacting to timer events, and HTTP events
    -------------------------------------------
    - Once running execute in a bash shell, and type the following to give HTTP events:

        curl -X 'POST'   'http://0.0.0.0:8000/api/event'   -d '{ "channel": "betfsm","event": "OPEN" }'

    - Possible events defined in this demo are OPEN, CLOSE, STOP, PLAYSOUND

    - api is documented using swagger: 'http://0.0.0.0:8000/docs'

    ALTERNATIVELY, open the advanced_example.html file in your browser



    """
    print(doc)
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("skill_example")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())   
    #set_logger("constructor",my_node.get_logger())  # useful for debugging, not useful to set in ROSRunner, 
                                                     # since ROSRunner runs after construction is complete !
    #set_logger("service",my_node.get_logger()) 
    #set_logger("state",my_node.get_logger())   


    get_logger().info("skill_example_1 started")
    blackboard = {}

    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)
    
    get_logger().info("Creating state machine: ")


    nominal_sm = ConcurrentSequence("concurrentseq", [
                        my_task(),
                        my_gripper_statemachine(),
                        EventSequential("check_play_sound", HTTPEvent_Condition(), {
                                 "PLAYSOUND": play_sound()
                                 })                            
                    ])
        

    # can't use twice the same subtree!
    sm = EventSequential("check", 
                         Ctrl_C_Condition("CTRL_C") | HTTPEvent_Condition() | Timeout_Condition("COMPLAIN",15,-1) | Timeout_Condition("QUIT",240),
                        event_map={
                            NO_EVENT: nominal_sm,
                            "CTRL_C": CrospiDeactivate(force_outcome="CANCEL"),
                            "STOP":   CrospiDeactivate(force_outcome="CANCEL"),
                            "COMPLAIN" : complain(),
                            "QUIT" : quitting()
                        }
    )

    
    runner = ROSRunner( my_node,
        sm,
         blackboard, 
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