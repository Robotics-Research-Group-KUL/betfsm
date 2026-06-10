# betfsm_etasl.py
#
# region Copyright (C) Erwin Aertbeliën, Santiago Iregui, 2024
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# endregion

"""
BeTFSM states related to eTaSL
"""
from typing import Callable
import json
import copy
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSHistoryPolicy,QoSReliabilityPolicy
from betfsm import (
    SUCCEED,CANCEL,TIMEOUT, TICKING,ABORT,NO_EVENT,
    add_logger_category, get_logger,Blackboard, TickingState,Message,Fallback,Sequence, Generator,
    CircularNumpyBuffer,get_path, EventOutcome,json_serializer
)
from betfsm_ros import (
    Node,Duration,
    BeTFSMNode,
    ServiceClient, Transition,ResetLifeCycleState,
    TopicEvent_Condition, LifeCycleTransition
)
from crospi_interfaces.srv import TaskSpecificationFile
from crospi_interfaces.srv import TaskSpecificationString
from crospi_interfaces.msg import Output




# Output:
#  - msg.names
#  - msg.data
#  - msg.is_declared    



from crospi_py import etasl_params

from rclpy.qos import QoSProfile


add_logger_category("crospi")


def load_task_list( json_file_name: str, blackboard: Blackboard) -> None:
    """
    Loads a task list from a file. References to packages and environment variables in the name
    are expanded (using the expand_... functions)

    Parameters:
        json_file_name: 
            json file containing the task list.
        blackboard:
            blackboard into which to load the task list.
    """
    etasl_params.load_task_list(json_file_name,blackboard)


def default_parameter_setter(state:TickingState,blackboard:Blackboard) -> dict:
    return {}


class SetTaskParameters(ServiceClient):
    def __init__(self,
                 name:str,
                 task_name:str,
                 srv_name:str = "/crospi_node",
                 cb: Callable[[TickingState,dict],dict] = default_parameter_setter,
                 timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None):
        """
        calls a service to instruct eTaSL to read a robot specification file. The file is obtained
        from the task

        Parameters:
            name:
                instance name
            task_name:
                name that will be used to find back the task with specifies the robot specification file.
            srv_name:
                name of the etasl node, by default `/crospi_node`
            cb:
                callback that sets the parameters, with signature `def param_setters(TickingState,Blackboard) ->param`
                where param is a Dict with the parameters of the task.  Every key in this dict will
                override the default parameters defined in the loaded json file with tasks.
            timeout:
                returns TIMEOUT if timeout is exceeded.
            node:
                ROS2 node, by default BeTFSMNode.get_instance()
        """        
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(name,srv_name+"/readTaskParameters",TaskSpecificationString,outcomes,timeout,node)
        self.task_name = task_name
        self.cb = cb

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        
        # task = etasl_params.get_task(blackboard,self.task_name)
        # # eliminate irrelevant parameters and put in a local dictionary:
        # # (parameter changes should be local, not global)
        # param = {}
        # for key,value in task["task_specification"]["parameters"].items():
        #     if (key[:3]!="is-") and (key!="file_path"):
        #         param[key] = value
        # param_definition = {}
        # param_definition.update(param)
        # # parameter checking using paramdef
        # for key,value in param.items():
        #     if key not in param_definition:
        #         raise ValueError(f"callback sets {key} parameter that is not in schema ")
        #     # not really needed anymore:
        #     if value == "external":
        #         raise ValueError(f"parameter declared 'external' is not set by callback ('external' is obsolete)")          

        param = copy.deepcopy(etasl_params.get_task_parameters_filled(blackboard,self.task_name))

        # calling callback
        if self.cb is not None:
            cbparam= self.cb(self,blackboard)
            for k,v in cbparam.items():
                param[k] = v

        request.str = json.dumps(param,default=json_serializer)
        get_logger("crospi").info(f"Set parameters for cROSpi task {self.task_name}: {request.str}")
        return request    
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            get_logger("crospi").error(f"SetTaskParameters failed for cROSpi task {self.task_name}")
            return ABORT


class ReadRobotSpecification(ServiceClient):
    def __init__(self,
                 name:str,
                 task_name:str,
                 srv_name:str = "/crospi_node",
                 timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None):
        """
        calls a service to instruct eTaSL to read a robot specification file. The file is obtained
        from the task

        Parameters:
            name:
                instance name
            task_name:
                name that will be used to find back the task with specifies the robot specification file.
            srv_name:
                name of the etasl node, by default `/crospi_node`
            timeout:
                returns TIMEOUT if timeout is exceeded.
            node:
                ROS2 node, by default BeTFSMNode.get_instance()
        """
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(
            name,
            srv_name+"/readRobotSpecification",
            TaskSpecificationFile, outcomes, timeout,node)
        self.task_name = task_name

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        request.file_path = etasl_params.get_robot_specification_for_task(blackboard, self.task_name)
        get_logger("crospi").info(f"robot specification file {request.file_path}")
        return request
            
        raise Exception(f"Task with name {self.task_name} was not found")
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            return ABORT


class ReadTaskSpecification(ServiceClient):
    def __init__(self,
                 name:str,
                 task_name:str,
                 srv_name:str = "/crospi_node",
                 timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None):
        """
        calls a service to instruct eTaSL to read a task specification file. The file is obtained
        from the task

        Parameters:
            task_name:
                name that will be used to find back the task with specifies the task specification file.
            srv_name:
                name of the etasl node, by default `/crospi_node`
            timeout:
                returns TIMEOUT if timeout is exceeded.
            node:
                ROS2 node, by default BeTFSMNode.get_instance()
        """        
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(
            name,
            srv_name+"/readTaskSpecificationFile",
            TaskSpecificationFile, outcomes, timeout,node)
        self.task_name = task_name

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        task = etasl_params.get_task(blackboard,self.task_name)
        # extract file_path, and expand references    
        # # we do expand refs, crospi_node does this already in his own ROS2 workspace    
        request.file_path = task["task_specification"]["file_path"]
        get_logger("crospi").info(f"task specification file {request.file_path}")
        return request
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            return ABORT


# region old eTaSLStateMachine code:
# class eTaSLOutput(TickingState):
#     def __init__(
#             self,
#             name:str,
#             topic: str,
#             qos:QoSProfile=30,
#             bb_location: str = "/output_param",
#             node: Node = None,
#         ) -> None:
#         """
#         At every tick processes the latest message from the topic and puts it 
#         on a predetermined location in the blackboard.

#         Parameters:
#             name:
#                 instance name
#             topic:
#                 topic to listen to, topics with interfacev `crospi_interfaces/msg/Output`
#                 are expected
#             qos:
#                 specification of the topic quality of service profile (QOSProfile)
#             bb_location:
#                 a location in the blackboard, written as a path, e.g. '/output_param'.
#                 The location has to exist when eTaSLOutput is first executed as a BeTFSM node
#             node:
#                 ROS2 node where the subscription will run.  if None, BeTFSMNode.get_instance() is used.

#         warning:
#             will only store messages where for all variables in the message is_declared is true.
#         """
#         deprecated_msg("Use CrospiOutput instead")
#         if node==None:
#             self.node = BeTFSMNode.get_instance()
#         else:
#             self.node = node
#         super().__init__("eTaSLOutput",[SUCCEED])
#         self.topic = topic
#         self.qos = qos
#         self.bb_location = bb_location
#         self.bb_loc = None

#     def cb_msg(self,msg) -> None:
#         # if reduce(and_,msg.is_declared):
#         # if reduce(and_,msg.is_declared, True): #TODO: Should this be used instead? (Santiago and Federico)
#         if all(msg.is_declared):
#             self.msgbuffer = msg

#     def entry(self,blackboard:Blackboard):
#         get_logger("crospi").info(f"eTaSLOutput: subscribing to topic {self.topic}")
#         self.bb_loc =  get_path_location(blackboard,self.bb_location)
#         self.msgbuffer=None
#         self.subscription = self.node.create_subscription(Output,self.topic,self.cb_msg,self.qos)
#         return TICKING
        
#     def doo(self,blackboard:Blackboard):
#         if self.msgbuffer is not None:
#             self.bb_loc.update( [ e for e in zip(self.msgbuffer.names,self.msgbuffer.data) ] )
#         return TICKING

#     def exit(self) -> str:
#         self.node.destroy_subscription(self.subscription)
#         return self.outcome


# class eTaSLEvent(TickingState):
#     """
#     !!! Error
#         This class is now obsolete, do **NOT** use. Use EventOutcome together 
#         with crospi_polling_func instead.  Additional posibilities using
#         EventSequential and EventConcurrent.

#     At every tick processes the latest message from the topic.
#     Node will CANCEL when it is ticked and no topic has been received.
#     """
#     def __init__(
#             self,
#             name: str,
#             topic: str = "/my_topic",
#             qos:QoSProfile=10,
#             mapping:Dict[str,tuple[int,str]]={"e_finished@crospi_node":(1,SUCCEED)},
#             node: Node = None,
#         ) -> None:
#         """
#         A node that waits until an event is received.
        
#         Parameters:
#             name:
#                 instance name
#             topic:
#                 name of the topic.  Topics of type String are expected.
#             qos:
#                 To be able to adapt the QoS
#             mapping:
#                 maps the event string to a priority and outcome.  
#                 When multiple messages are received, the event with lowest priority
#                 will be used.  For events with the same priority the earliest
#                 event will be selected.
#             node:
#                 node or BeTFSMNode if node==None.
#         """
#         deprecated_msg("Use CheckOutcome with crospi_polling_func(...) instead")
#         if node==None:
#             self.node = BeTFSMNode.get_instance()
#         else:
#             self.node = node
#         outcomes=[]
#         for k,v in mapping.items():
#             outcomes.append( v[1]  )
#         super().__init__(name,outcomes)
#         self.topic = topic
#         self.qos = qos
#         self.mapping = mapping
#         self.buffer = None
#         self.count = 0

#     def cb_msg(self,msg) -> None:
#         r = self.mapping.get( msg.data)
#         newr =  (r[0],self.count,r[1]) 
#         self.count += 1
#         if self.buffer is not None:
#             if newr < self.buffer:
#                 self.buffer = newr
#         else:
#             self.buffer = newr

#     def entry(self,blackboard:Blackboard):   
#         self.buffer=None
#         self.count=0
#         self.subscription = self.node.create_subscription(String,self.topic,self.cb_msg,self.qos)
#         return TICKING
        
#     def doo(self,blackboard:Blackboard):
#         # return the outcome belonging to the first message with an event in mapping.
#         if self.buffer is not None:
#             return self.buffer[2]
#         else:
#             return TICKING

#     def exit(self) -> str:
#         self.node.destroy_subscription(self.subscription)
#         return self.outcome


# class eTaSL_StateMachine(TickingStateMachine):
#     """
#     """
#     def __init__(self,
#                  name : str,
#                  task_name: str,
#                  srv_name: str = "/crospi_node",
#                  output_topic: str = "/my_topic",
#                  event_topic: str = "crospi_node/events",
#                  #display_in_viewer: bool= False, 
#                  cb:Callable=default_parameter_setter,
#                  timeout:Duration = Duration(seconds=1.0),
#                  node : Node = None,
#                  deactivate_last: bool = False
#                  ):
#         """
#         Configurable statemachine to execute an eTaSL task that:

#         - uses TickingStateMachine to provide callbacks for transtions and state changes and support TICKING;
#         - uses a feedback to set parameters;
#         - puts the last message from the output topic in the blackboard under `blackboard["output"][name]`.

#         Parameters:
#             name:
#                 name of this state machine (i.e. task instance)
#             task_name:
#                 name of the task to be executed (i.e. task type) Will be looked up in the blackboard.
#             srv_name:
#                 name of the eTaSL node, by default /crospi_node
#             output_topic:
#                 name of the topic where to find the output of eTaSL. A topic with interface *crospi_interfaces/msg/Output* is expected.
#             cb:
#                 callback that sets the parameters, with signature `def cb(blackboard) ->param`
#                 where param is a Dict with the parameters of the task that will be used to update
#                 the default parameters.
#             timeout:
#                 [optional] returns TIMEOUT if the communication timeout of any of the substeps is exceeded. 
#                 Uses a duration of 1 second otherwise.
#             node:
#                 [optional] ROS2 node to be used. Uses BeTFSMNode.get_instance() otherwise.

#         warning:            
#             TODO: default name of output topic needs to be changed.
#         """
#         deprecated_msg("Use CrospiTask instead")
#         super().__init__(name,outcomes=[SUCCEED, ABORT,TIMEOUT]) # removed parameters: ,transitioncb=transitioncb,statecb=statecb)
#         msg = Message(name="display_name", msg=f"cROSpi task {name}", logCategory="crospi")
#         self.add_state(msg,transitions={SUCCEED: "DEACTIVATE_ETASL"})
#         self.set_start_state(msg)
#         self.add_state(LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node),
#                     transitions={SUCCEED: "CLEANUP_ETASL",
#                                 ABORT: "CLEANUP_ETASL",
#                                 TIMEOUT: ABORT}) 
#         self.add_state(LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node),
#                     transitions={SUCCEED: "PARAMETER_CONFIG",
#                     ABORT: "PARAMETER_CONFIG"}) 
#         self.add_state(SetTaskParameters( "PARAMETER_CONFIG",task_name, srv_name, cb, timeout, node ),
#                        transitions={
#                            SUCCEED: "ROBOT_SPECIFICATION" })
#         self.add_state(ReadRobotSpecification("ROBOT_SPECIFICATION",task_name,srv_name,timeout,node),
#                 transitions={
#                     SUCCEED: "TASK_SPECIFICATION"})

#         self.add_state(ReadTaskSpecification("TASK_SPECIFICATION",task_name,srv_name,timeout,node),
#                 transitions={
#                     SUCCEED: "CONFIG_ETASL"})

#         self.add_state(LifeCycle("CONFIG_ETASL",srv_name,Transition.CONFIGURE,timeout,node),
#                 transitions={
#                     SUCCEED: "ACTIVATE_ETASL"})

#         self.add_state(LifeCycle("ACTIVATE_ETASL",srv_name,Transition.ACTIVATE,timeout,node),
#                 transitions={
#                     SUCCEED: "EXECUTING"})

#         # executes until one returns SUCCEED,  eTaSLOutput only returns TICKING
#         mapping={"e_finished@{}".format(srv_name[1:]):(1,SUCCEED)}
#         if not deactivate_last:
#             transition_map_executing = {}
#         else:
#             transition_map_executing = {SUCCEED: "DEACTIVATE_ETASL_LAST"}
            
#         self.add_state(
#             ConcurrentFallback("EXECUTING",[
#                 eTaSLEvent(name="check_event",topic=event_topic, mapping=mapping,node=node),
#                 eTaSLOutput("output", topic=output_topic, bb_location=f"/output_param/{name}", node=node)
#             ]),
#             transitions=transition_map_executing
#         )

#         if deactivate_last:
#             self.add_state(LifeCycle("DEACTIVATE_ETASL_LAST",srv_name,Transition.DEACTIVATE,timeout,node),
#                     transitions={SUCCEED: "CLEANUP_ETASL_LAST",
#                                 ABORT: "CLEANUP_ETASL_LAST",
#                                 TIMEOUT: ABORT})
#             self.add_state(LifeCycle("CLEANUP_ETASL_LAST",srv_name,Transition.CLEANUP,timeout,node),
#                     transitions={SUCCEED: SUCCEED,
#                                  ABORT: ABORT,
#                                  TIMEOUT: ABORT}) 
# endregion

#region  class CrospiDeactivate(Sequence):    
#     def __init__(self, srv_name: str = "/crospi_node",
#                  timeout:Duration = Duration(seconds=1.0),
#                  node : Node = None, force_outcome:str=None):
#         """ 
#         Sets lifecycle cROSpi to UNCONFIGURED state (if not already in that state)
        
#         Always tries to go true the whole sequence, also in case of errors.  Depending
#         on the context (cleanup, or initial check), you can use `force_outcome` to force the outcome to
#         a given value.

#         Parameters:
#             srv_name:
#                 name of the crospi_node
#             timeout:
#                 duration of timeout
#             node:
#                 ROS2 node that BeTFSM uses (None: will get singleton instance)
#             force_outcome:
#                 the outcome of this statemachine (in all circumstances)
#         """
#         super().__init__("crospi_deactivate")
        
#         deactivate_end  = LifeCycle("DEACTIVATE_CROSPI",srv_name,Transition.DEACTIVATE,timeout,node,always_succeed=True)
#         cleanup_end     = LifeCycle("CLEANUP_CROSPI",srv_name,Transition.CLEANUP,timeout,node,always_succeed=True)

#         self.add_state(deactivate_end)
#         self.add_state(cleanup_end) 
#         if force_outcome is not None:
#             self.add_state( AlwaysOutcome(force_outcome) )
#endregion

class CrospiDeactivate(ResetLifeCycleState):
    """ 
    Sets lifecycle cROSpi to UNCONFIGURED state (if not already in that state)
    """
    def __init__(self, 
                 srv_name: str = "/crospi_node",
                 timeout: Duration = Duration(seconds=1.0),
                 node: BeTFSMNode=None,
                 force_outcome:str=None):
        """ 
        Sets lifecycle cROSpi to UNCONFIGURED state (if not already in that state)
        
        Always tries to go true the whole sequence, also in case of errors.  Depending
        on the context (cleanup, or initial check), you can use `force_outcome` to force the outcome to
        a given value.

        Parameters:
            srv_name:
                name of the crospi_node
            timeout:
                duration of timeout
            node:
                ROS2 node that BeTFSM uses (None: will get singleton instance)
            force_outcome:
                the outcome of this statemachine (in all circumstances)
        """
        super().__init__("reset",srv_name,timeout,node)
        self.force_outcome = force_outcome

    def exit(self):
        outcome= super().exit()
        if self.force_outcome is None:
            return outcome
        else:
            return self.force_outcome
        

class CrospiTask(Fallback):

    def cb_param(self, state, path:str):
        return self.cb(self,path)


    def __init__(self, 
                 name : str,
                 task_name: str,
                 srv_name: str = "/crospi_node",
                 cb:Callable[[TickingState,dict],dict]=default_parameter_setter,
                 event_topic: str = "crospi_node/events",
                 timeout:Duration = Duration(seconds=1.0),
                 node : Node = None,
                 eventqueue_size: int = 10,
                 max_age: float = 0.1,
                 event_check: bool = True
                 ):
        """
        returns a subtree that starts an eTaSL task on a cROSpi node.  The first two
        arguments are the most important, all the others have reasonable default
        settings.  the event_check parameter is useful when you just want to startup
        the Crospi task and call CrospiDeactivate(...) yourself.

        This tasks still starts by calling deactivate and cleanup on the crospy lifecycle.
        This takes care of Crospi being in the wrong state of its lifecycle due to a previous
        run of another application with an error.

        Parameters:
            name:
                name of this state machine (i.e. task instance)
            task_name:
                name of the task to be executed (i.e. task type) Will be looked up in the blackboard.
            srv_name:
                name of the eTaSL node, by default /crospi_node
            cb:
                callback that sets the parameters, with signature `def cb(TickingState,Blackboard) ->param`
                where param is a Dict with the parameters of the task that will be used to update
                the default parameters.
            event_topic:
                topic where the events are read.  Topic should be of type std_msgs/msg/string
            timeout:
                [optional] returns TIMEOUT if the communication timeout of any of the substeps is exceeded. 
                Uses a duration of 1 second otherwise.
            node:
                [optional] ROS2 node to be used. Uses BeTFSMNode.get_instance() otherwise.
            eventqueue_size:
                minimum size of the event queue that receives crospi events.
            max_age:
                maximum age in seconds that is allowed to be received.
            event_check:
                when false, just starts the crospi task but do not check for e_finished event or deactivate/cleanup crospi
                otherwise wait while checking and deactivate/cleanup crospi afterwards.

        """       
        # Adapt cb such that possible path local references are refering to CrospiTask and not the inner SetTaskParameters 
        #def cb_parent(state,bb):
            #print("cb_parent\nstate",state)
            #print("state.parent ", state.parent)
            #print("self: ",self)
            #return cb(self,bb)
        self.cb = cb
        # e_finished is always handled.  The rest should be handled outside
        event_mapping={ NO_EVENT: TICKING,f"e_finished@{srv_name[1:]}" : SUCCEED}
        # Note: outcomes in mapping will automatically be added to outcomes of checkevent and crospiRun node
        msg_start         = Message(name=name,msg=f"crospiTask {name} has started",logCategory="crospi")
        # not really needed, except for a previous run that leaves crospi in wrong state:
        reset             = ResetLifeCycleState("Reset_Lifecycle", srv_name,timeout,node) 
        # set parameters, robot and task spec
        settaskparam      = SetTaskParameters( "SetTaskParameters",task_name, srv_name, self.cb_param, timeout, node )
        readrobotspec     = ReadRobotSpecification("ReadRobotSpec",task_name,srv_name,timeout,node)
        readtaskspec      = ReadTaskSpecification("ReadTaskSpec",task_name,srv_name,timeout,node)
        configure         = LifeCycleTransition("Configure_Crospi",srv_name,Transition.CONFIGURE,timeout,node)
        activate          = LifeCycleTransition("Activate_Crospi",srv_name,Transition.ACTIVATE,timeout,node)

        seq               = Sequence(name+"_seq",[
                                msg_start, 
                                reset,
                                #deactivate_start,cleanup_start,
                                settaskparam,readrobotspec,readtaskspec, 
                                configure, activate
                            ])                
        if event_check:
            checkevent        = EventOutcome("checkevent",TopicEvent_Condition(node,event_topic,eventqueue_size,max_age),event_mapping)        
            reset_end         = ResetLifeCycleState("Reset_Lifecycle", srv_name,timeout,node) 
            msg_end           = Message(name=name,msg=f"crospiTask {name} has finished, crospi is deactivated",logCategory="crospi")
            seq.add_state(checkevent)
            seq.add_state(reset_end)
            seq.add_state(msg_end)
        else: 
            msg_end           = Message(name=name,msg=f"crospiTask {name} finished, crospi will keep running",logCategory="crospi")
            seq.add_state(msg_end)
        
        cleanup_seq = CrospiDeactivate(srv_name,timeout,node,force_outcome=CANCEL)        
        # if the seq ends with ABORT or TIMEOUT, fallback to cleanup_seq
        super().__init__(name,[seq,cleanup_seq],lambda bb,oc: oc==ABORT or oc==TIMEOUT)


class CrospiOutput(Generator):
    """
    Record output of Crospi in a topic while executing subtree
    """
    def __init__(
            self,
            name:str,
            topic: str,
            queue_size: int = 1_000_000,
            path: str = "../output",            
            node: Node = None,
        ) -> None:
        """Record output of Crospi for a topic in a queue while executing subtree
           This queue is put in a specified location in the blackboard under two children "header" and "data".  The header is
           taken from the labels of the first message that arrives.


        Parameters
        ----------
        name : str
            name of the node
        topic : str
            ROS2 topic to subscribe to, should be of message type Output
        queue_size : int, optional
            maximum length of the buffer to record the data, by default 1_000_000
        path : str, optional
            path inside the blackboard, by default "/output"
        node : Node, optional
            ROS2 node, if None, BeTFSMNode.get_instance() will be used. by default None
        """
        super().__init__("eTaSLOutput",[SUCCEED, CANCEL])
        if node==None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        self.topic = topic
        self.qos = QoSProfile(
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.VOLATILE
        )
        self.path       = path
        self.queue_size = queue_size

    def cb_msg(self,msg) -> None:
        if len(msg.is_declared)>0 and all(msg.is_declared):
            if self.first_time:
                #self.header.append([n for n in msg.names ])
                self.header[:]   = [n for n in msg.names ]
                self.queue               = CircularNumpyBuffer(self.queue_size,len(self.header))
                self.path_ns['data']     = self.queue   # path_ns['data'] and queue point to the same data
                self.first_time          = False
            self.queue.add( [d for d in msg.data ])

            

    def co_execute(self,blackboard):
        self.path_ns               = get_path(self,blackboard,self.path)
        self.header           = []        
        self.queue            = None         # don't know the number of columns yet
        self.path_ns["data"]  = self.queue
        self.path_ns["header"]= self.header
        self.first_time       = True
        self.subscription     = self.node.create_subscription(Output,self.topic,self.cb_msg,self.qos)
        while True:
            yield TICKING        

    def exit(self):
        # this is also called when its externally reset or naturally finishes by itself
        self.node.destroy_subscription(self.subscription)
        if self.queue is not None and len(self.queue)>0:
            loc = {}
            self.path_ns["last"] = loc
            self.path_ns["is-circularbuffer"] = True
            self.path_ns["capacity"]          = self.queue.get_capacity() 
            last = self.queue.peek_newest()
            for lbl,val in zip(self.header, last):
                loc[lbl] = val
        return super().exit()

