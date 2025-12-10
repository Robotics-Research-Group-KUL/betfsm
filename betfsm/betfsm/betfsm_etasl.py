# betfsm_etasl.py
#
# Copyright (C) Erwin Aertbeliën, Santiago Iregui, 2024
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


"""
BeTFSM states related to eTaSL
"""
from rclpy.qos import QoSProfile

from .betfsm_ros import *

from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString

from etasl_ros2_py import etasl_params


import json
from jsonschema import validate, exceptions

# Output:
#  - msg.names
#  - msg.data
#  - msg.is_declared    
from etasl_interfaces.msg import Output
    

from std_msgs.msg import String
from functools import reduce
from operator import and_

        # qos_profile = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST, #Keeps the last msgs received in case buffer is fulll
        #     depth=msg_queue, #Buffer size
        #     reliability=QoSReliabilityPolicy.RELIABLE, #Uses TCP for relia
        # bility instead of UDP
        #     durability=QoSDurabilityPolicy.VOLATILE #Volatile, may not use first msgs if subscribed late (will not happen in this context)
        # )  



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


def default_parameter_setter(blackboard):
    return {}

class SetTaskParameters(ServiceClient):
    def __init__(self,
                 name:str,
                 task_name:str,
                 srv_name:str = "/etasl_node",
                 cb: Callable = default_parameter_setter,
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
                name of the etasl node, by default `/etasl_node`
            cb:
                callback that sets the parameters, with signature `def param_setters(blackboard) ->param`
                where param is a Dict with the parameters of the task.
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

        param = etasl_params.get_task_parameters_filled(blackboard,self.task_name)

        # calling callback
        if self.cb is not None:
            param.update( self.cb(blackboard)  )

        request.str = json.dumps(param)
        get_logger().info(f"Set parameters for eTaSL task {self.task_name}\n{request.str}")
        return request    
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            return ABORT




class ReadRobotSpecification(ServiceClient):
    def __init__(self,
                 name:str,
                 task_name:str,
                 srv_name:str = "/etasl_node",
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
                name of the etasl node, by default `/etasl_node`
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
                 srv_name:str = "/etasl_node",
                 timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None):
        """
        calls a service to instruct eTaSL to read a task specification file. The file is obtained
        from the task

        Parameters:
            task_name:
                name that will be used to find back the task with specifies the task specification file.
            srv_name:
                name of the etasl node, by default `/etasl_node`
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
        # # we do expand refs, etasl_node does this already in his own ROS2 workspace    
        request.file_path = task["task_specification"]["file_path"]
        get_logger().info(f"task specification file {request.file_path}")
        return request
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            return ABORT



class eTaSLOutput(TickingState):
    def __init__(
            self,
            name:str,
            topic: str,
            qos:QoSProfile=30,
            bb_location: List = ["output_param"],
            node: Node = None,
        ) -> None:
        """
        At every tick processes the latest message from the topic and puts it 
        on a predetermined location in the blackboard.

        Parameters:
            name:
                instance name
            topic:
                topic to listen to, topics with interfacev `etasl_interfaces/msg/Output`
                are expected
            qos:
                specification of the topic quality of service profile (QOSProfile)
            bb_location:
                list of strings indicating location in the blackboard.
            node:
                ROS2 node where the subscription will run.  if None, BeTFSMNode.get_instance() is used.

        warning:
            will only store messages where for all variables in the message is_declared is true.
        """
        if node==None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        super().__init__("eTaSLOutput",[SUCCEED])
        self.topic = topic
        self.qos = qos
        self.bb_location = bb_location

    def get_location(self,blackboard):
        bb = blackboard
        for key in self.bb_location:
            if (key in bb) and isinstance(bb[key],Dict):
                bb = bb[key]
            else:
                bb[key] = {}
                bb      = bb[key]
        return bb

    def cb_msg(self,msg) -> None:
        # if reduce(and_,msg.is_declared):
        if reduce(and_,msg.is_declared, True): #TODO: Should this be used instead? (Santiago and Federico)
            self.msgbuffer = msg

    def entry(self,blackboard:Blackboard):      
        self.msgbuffer=None
        self.subscription = self.node.create_subscription(Output,self.topic,self.cb_msg,self.qos)
        return TICKING
        
    def doo(self,blackboard:Blackboard):
        if self.msgbuffer is not None:
            bb = self.get_location(blackboard)
            bb.update( [ e for e in zip(self.msgbuffer.names,self.msgbuffer.data) ] )
        return TICKING

    def exit(self) -> str:
        self.node.destroy_subscription(self.subscription)
        return self.outcome


class eTaSLEvent(TickingState):
    """
    At every tick processes the latest message from the topic.
    Node will CANCEL when it is ticked and no topic has been received.
    """
    def __init__(
            self,
            name: str,
            topic: str,
            qos:QoSProfile=10,
            mapping:Dict[str,tuple[int,str]]={"e_finished@etasl_node":(1,SUCCEED)},
            node: Node = None,
        ) -> None:
        """
        A node that waits until an event is received.
        
        Parameters:
            name:
                instance name
            topic:
                name of the topic.  Topics of type String are expected.
            qos:
                To be able to adapt the QoS
            mapping:
                maps the event string to a priority and outcome.  
                When multiple messages are received, the event with lowest priority
                will be used.  For events with the same priority the earliest
                event will be selected.
            node:
                node or BeTFSMNode if node==None.
        """
        if node==None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        outcomes=[]
        for k,v in mapping.items():
            outcomes.append( v[1]  )
        super().__init__(name,outcomes)
        self.topic = topic
        self.qos = qos
        self.mapping = mapping
        self.buffer = None
        self.count = 0

    def cb_msg(self,msg) -> None:
        r = self.mapping.get( msg.data)
        newr =  (r[0],self.count,r[1]) 
        self.count += 1
        if self.buffer is not None:
            if newr < self.buffer:
                self.buffer = newr
        else:
            self.buffer = newr

    def entry(self,blackboard:Blackboard):   
        self.buffer=None
        self.count=0
        self.subscription = self.node.create_subscription(String,self.topic,self.cb_msg,self.qos)
        return TICKING
        
    def doo(self,blackboard:Blackboard):
        # return the outcome belonging to the first message with an event in mapping.
        if self.buffer is not None:
            return self.buffer[2]
        else:
            return TICKING

    def exit(self) -> str:
        self.node.destroy_subscription(self.subscription)
        return self.outcome


# def nested_etasl_state(name: str, file_path: str, robot_path: str, display_in_viewer: bool= False):
class eTaSL_StateMachine(TickingStateMachine):
    """


    """
    def __init__(self,
                 name : str,
                 task_name: str,
                 srv_name: str = "/etasl_node",
                 output_topic: str = "/my_topic",
                 event_topic: str = "/etasl/events",
                 #display_in_viewer: bool= False, 
                 cb:Callable=default_parameter_setter,
                 timeout:Duration = Duration(seconds=1.0),
                 node : Node = None,
                 deactivate_last: bool = False,
                 transitioncb:Callable=default_transitioncb, 
                 statecb:Callable=default_statecb
                 ):
        """
        Configurable statemachine to execute an eTaSL task that:

        - uses TickingStateMachine to provide callbacks for transtions and state changes and support TICKING;
        - uses a feedback to set parameters;
        - puts the last message from the output topic in the blackboard under blackboard["output"][name].

        Parameters:
            name:
                name of this state machine (i.e. task instance)
            task_name:
                name of the task to be executed (i.e. task type) Will be looked up in the blackboard.
            srv_name:
                name of the eTaSL node, by default /etasl_node
            output_topic:
                name of the topic where to find the output of eTaSL. A topic with interface *etasl_interfaces/msg/Output* is expected.
            cb:
                callback that sets the parameters, with signature `def cb(blackboard) ->param`
                where param is a Dict with the parameters of the task that will be used to update
                the default parameters.
            timeout:
                [optional] returns TIMEOUT if the communication timeout of any of the substeps is exceeded. 
                Uses a duration of 1 second otherwise.
            node:
                [optional] ROS2 node to be used. Uses BeTFSMNode.get_instance() otherwise.
            transitioncb:
                [optional] callback that is called at each transition, signature `def transitioncb(statemachine,blackboard,source,outcome)->outcome`
            statecb:
                [optional] callback that is called at each, signature `default_statecb(statemachine,blackboard,state)`

        warning:            
            TODO: name of output topic needs to be changed.
        """
        super().__init__(name,outcomes=[SUCCEED, ABORT,TIMEOUT],transitioncb=transitioncb,statecb=statecb)

        self.add_state(LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node),
                    transitions={SUCCEED: "CLEANUP_ETASL",
                                ABORT: "CLEANUP_ETASL",
                                TIMEOUT: ABORT}) 
        self.add_state(LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node),
                    transitions={SUCCEED: "PARAMETER_CONFIG",
                    ABORT: "PARAMETER_CONFIG"}) 
        self.add_state(SetTaskParameters( "PARAMETER_CONFIG",task_name, srv_name, cb, timeout, node ),
                       transitions={
                           SUCCEED: "ROBOT_SPECIFICATION",
                           ABORT: ABORT
                       })
        self.add_state(ReadRobotSpecification("ROBOT_SPECIFICATION",task_name,srv_name,timeout,node),
                transitions={
                    SUCCEED: "TASK_SPECIFICATION",
                    ABORT: ABORT})

        self.add_state(ReadTaskSpecification("TASK_SPECIFICATION",task_name,srv_name,timeout,node),
                transitions={
                    SUCCEED: "CONFIG_ETASL",
                    ABORT: ABORT})

        self.add_state(LifeCycle("CONFIG_ETASL",srv_name,Transition.CONFIGURE,timeout,node),
                transitions={
                    SUCCEED: "ACTIVATE_ETASL",
                    ABORT: ABORT})

        self.add_state(LifeCycle("ACTIVATE_ETASL",srv_name,Transition.ACTIVATE,timeout,node),
                transitions={
                    SUCCEED: "EXECUTING",
                    ABORT: ABORT})

        # executes until one returns SUCCEED,  eTaSLOutput only returns TICKING
        mapping={"e_finished@{}".format(srv_name[1:]):(1,SUCCEED)}
        if not deactivate_last:
            transition_map_executing = {SUCCEED:SUCCEED}
            
        else:
            transition_map_executing = {
                SUCCEED: "DEACTIVATE_ETASL_LAST"
            }
            
        self.add_state(
            ConcurrentFallback("EXECUTING",[
                eTaSLEvent(name="check_event",topic=event_topic, mapping=mapping,node=node),
                eTaSLOutput("output", topic=output_topic, bb_location=["output_param",name], node=node)
            ]),
            transitions=transition_map_executing
        )

        if deactivate_last:
            self.add_state(LifeCycle("DEACTIVATE_ETASL_LAST",srv_name,Transition.DEACTIVATE,timeout,node),
                    transitions={SUCCEED: "CLEANUP_ETASL_LAST",
                                ABORT: "CLEANUP_ETASL_LAST",
                                TIMEOUT: ABORT})
            self.add_state(LifeCycle("CLEANUP_ETASL_LAST",srv_name,Transition.CLEANUP,timeout,node),
                    transitions={SUCCEED: SUCCEED,
                                 ABORT: ABORT,
                                 TIMEOUT: ABORT}) 
