from betfsm import (
    SUCCEED,CANCEL,TIMEOUT, TICKING,ABORT,NO_EVENT,
    Message, Fallback,Sequence, AlwaysOutcome,
    EventOutcome
)

from betfsm_ros import (
    Node,Duration,
        LifeCycle, Transition
)
from betfsm_crospi import (
    default_parameter_setter,
    SetTaskParameters,ReadRobotSpecification,ReadTaskSpecification,eTaSLOutput,
    crospi_polling_func
)

from typing import Dict, List, Union, Callable,Type, TypeAlias


class CrospiDeactivate(Sequence):    
    def __init__(self, srv_name: str = "/crospi_node",
                 timeout:Duration = Duration(seconds=1.0),
                 node : Node = None, force_outcome:str=None):
        """ 
        Sets lifecycle cROSpi to UNCONFIGURED state (if not already in that state)
        Always tries to go true the whole sequence, also in case of errors.

        Parameters:
            srv_name:
                name of the crospi_node
            timeout:
                duration of timeout
            node:
                ROS2 node that BeTFSM uses (None: will get singleton instance)
            outcome:
                the outcome of this statemachine (in all circumstances)
        """
        super().__init__("crospi_stop")
        
        deactivate_end  = LifeCycle("DEACTIVATE_CROSPI",srv_name,Transition.DEACTIVATE,timeout,node,always_succeed=True)
        cleanup_end     = LifeCycle("CLEANUP_CROSPI",srv_name,Transition.CLEANUP,timeout,node,always_succeed=True)

        self.add_state(deactivate_end)
        self.add_state(cleanup_end) 
        if force_outcome is not None:
            self.add_state( AlwaysOutcome(force_outcome) )
        

class CrospiTask(Fallback):
    def __init__(self, 
                 name : str,
                 task_name: str,
                 srv_name: str = "/crospi_node",
                 cb:Callable=default_parameter_setter,
                 event_topic: str = "crospi_node/events",
                 timeout:Duration = Duration(seconds=1.0),
                 node : Node = None,
                 eventqueue_size: int = 10,
                 max_age: float = 0.1,
                 event_check: bool = True
                 ):
        """
        returns a subtree that starts an eTaSL task on a cROSpi node.

        Parameters:
            name:
                name of this state machine (i.e. task instance)
            task_name:
                name of the task to be executed (i.e. task type) Will be looked up in the blackboard.
            srv_name:
                name of the eTaSL node, by default /crospi_node
            cb:
                callback that sets the parameters, with signature `def cb(blackboard) ->param`
                where param is a Dict with the parameters of the task that will be used to update
                the default parameters.
            event_topic:
                topic where the events are read.  Topic should be of type std_msgs/msg/string
            timeout:
                [optional] returns TIMEOUT if the communication timeout of any of the substeps is exceeded. 
                Uses a duration of 1 second otherwise.
            node:
                [optional] ROS2 node to be used. Uses BeTFSMNode.get_instance() otherwise.
            eventqeueu_size:
                minimum size of the event queue
            max_age:
                maximum age in seconds that is allowed to be received.
            event_check:
                when false, just starts the crospi task but do not check for e_finished event or deactivate/cleanup crospi
                otherwise wait while checking and deactivate/cleanup crospi afterwards.

        """        
        # e_finished is always handled.  The rest should be handled outside
        event_mapping={ NO_EVENT: TICKING,f"e_finished@{srv_name[1:]}" : SUCCEED}
        # Note: outcomes in mapping will automatically be added to outcomes of checkevent and crospiRun node
        msg_start         = Message(name=name,msg="crospiTask {name} has started")
        settaskparam      = SetTaskParameters( "SetTaskParameters",task_name, srv_name, cb, timeout, node )
        readrobotspec     = ReadRobotSpecification("ReadRobotSpec",task_name,srv_name,timeout,node)
        readtaskspec      = ReadTaskSpecification("ReadTaskSpec",task_name,srv_name,timeout,node)
        configure         = LifeCycle("CONFIGURE_CROSPI",srv_name,Transition.CONFIGURE,timeout,node)
        activate          = LifeCycle("ACTIVATE_CROSPI",srv_name,Transition.ACTIVATE,timeout,node)

        seq               = Sequence(name+"_seq",[
                                msg_start, settaskparam,readrobotspec,readtaskspec, 
                                configure, activate
                            ])                
        if event_check:
            checkevent        = EventOutcome("checkevent",crospi_polling_func(node,event_topic,eventqueue_size,max_age=max_age),event_mapping)        
            deactivate_end    = LifeCycle("DEACTIVATE_CROSPI",srv_name,Transition.DEACTIVATE,timeout,node,always_succeed=True)
            cleanup_end       = LifeCycle("CLEANUP_CROSPI",srv_name,Transition.CLEANUP,timeout,node,always_succeed=True)
            msg_end           = Message(name=name,msg="crospiTask {name} has finished, crospi is deactivated")
            seq.add_state(checkevent)
            seq.add_state(deactivate_end)
            seq.add_state(cleanup_end)
            seq.add_state(msg_end)
        else: 
            msg_end           = Message(name=name,msg="crospiTask {name} finished, crospi will keep running")
            seq.add_state(msg_end)
        
        cleanup_seq = CrospiDeactivate(srv_name,timeout,node,force_outcome=CANCEL)        
        # if the seq ends with ABORT or TIMEOUT, fallback to cleanup_seq
        super().__init__(name+"_fallback",[seq,cleanup_seq],lambda bb,oc: oc==ABORT or oc==TIMEOUT)

 

# class Example_user(TickingStateMachine):
#     def __init__(self):
#         super().__init__("my_example_advanced",[SUCCEED, CANCEL])
#         startcrospi      = CrospiStart("MovingSpline","MovingSpline")

#         mapping = {"e_finished@crospi_node":(1, SUCCEED),
#                    "e_gripper@crospi_node" :(2, "CLOSEBY")}
#         check            = eTaSLEvent("check",topic="/my_topic",mapping=mapping)
#         opengripper = Message(msg="Sending command to open gripper")
#         stopcrospi       = CrospiStop()

#         self.add_state(startcrospi, transitions={SUCCEED: check})
#         self.add_state(check, transitions={SUCCEED:stopcrospi,"CLOSEBY":opengripper})
#         self.add_state(opengripper, transitions={SUCCEED:check})

