# yasmin_ticking_etasl.py
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
Yasmin_Ticking states related to eTaSL
"""
from rclpy.qos import QoSProfile
from yasmin import Blackboard
from yasmin.blackboard import Blackboard

from .yasmin_ticking_ros import *

from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString


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


def get_task(blackboard:Blackboard,task_name:str) -> List[str|List]:
    """
    get the default parameters for a task.

    Parameters:
        blackboard: blackboard
        task_name: name of the task
    """
    try:
        tasks=blackboard["tasks"]
    except:
        raise Exception("There is no `tasks` defined in the blackboard, are you sure you loaded a list of tasks?")
    for task in tasks:
        if task.get("name")==task_name:
            if not("parameters" in task):
                raise Exception("task dictionary should have `parameters` keyword")
            return task
    raise Exception(f"No task with name {task_name} found in the blackboard")





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
    with open(expand_ref(json_file_name), 'r') as json_file:
        parameters = json.load(json_file)
        blackboard["tasks"] = parameters["tasks"]


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
                ROS2 node, by default YasminNode.get_instance()
        """        
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(name,srv_name+"/readTaskSpecificationString",TaskSpecificationString,outcomes,timeout,node)
        self.task_name = task_name
        self.cb = cb

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        task = get_task(blackboard,self.task_name)
        # eliminate irrelevant parameters and put in a local dictionary:
        # (parameter changes should be local, not global)
        param = {}
        for key,value in task["parameters"].items():
            if (key[:3]!="is-") and (key!="file_path"):
                param[key] = value
        param_definition = {}
        param_definition.update(param)
        # calling callback
        if self.cb is not None:
            param.update( self.cb(blackboard)  )
        # parameter checking using paramdef
        for key,value in param.items():
            if key not in param_definition:
                raise ValueError(f"callback sets {key} parameter that is not in schema ")
            # not really needed anymore:
            if value == "external":
                raise ValueError(f"parameter declared 'external' is not set by callback ('external' is obsolete)")          
        # constructing LUA script fragment:       
        param_string = ""
        for key, value in param.items():
            if isinstance(value,bool):
                param_string = param_string + f"{key} = {str(value).lower()}\n"
            elif isinstance(value,str):
                 param_string = param_string + f'{key} = "{value}"\n'
            else:
                param_string = param_string + f"{key} = {value}\n"
        # constructing request:
        request.str  = param_string
        get_logger().info(f"Set parameters for eTaSL task {self.task_name}\n{param_string}")
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
                ROS2 node, by default YasminNode.get_instance()
        """
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(
            name,
            srv_name+"/readTaskSpecificationFile",
            TaskSpecificationFile, outcomes, timeout,node)
        self.task_name = task_name

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        for task in blackboard["tasks"]:
            if task["name"]==self.task_name:
                specfile = task.get("robot_specification_file")
                if specfile is None or specfile=="":
                    specfile = blackboard["default_robot_specification"]
                    if specfile is None or specfile=="":
                        raise Exception("No robot_specification_file defined")
                # we do expand refs, etasl_node does this already in his own ROS2 workspace
                request.file_path  = specfile
                get_logger().info(f"robot specification file {specfile}")
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
                ROS2 node, by default YasminNode.get_instance()
        """        
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(
            name,
            srv_name+"/readTaskSpecificationFile",
            TaskSpecificationFile, outcomes, timeout,node)
        self.task_name = task_name

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        task = get_task(blackboard,self.task_name)
        # extract file_path, and expand references    
        # # we do expand refs, etasl_node does this already in his own ROS2 workspace    
        request.file_path = task["parameters"]["file_path"]
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
            bb_location: List = ["output"],
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
                ROS2 node where the subscription will run.  if None, YasminTicking.get_instance() is used.

        warning:
            will only store messages where for all variables in the message is_declared is true.
        """
        if node==None:
            self.node = YasminTickingNode.get_instance()
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
        if reduce(and_,msg.is_declared):
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
                node or YasminTickingNode if node==None.
        """
        if node==None:
            self.node = YasminTickingNode.get_instance()
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
                 #display_in_viewer: bool= False, 
                 cb:Callable=default_parameter_setter,
                 timeout:Duration = Duration(seconds=1.0),
                 node : Node = None,
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
                [optional] ROS2 node to be used. Uses YasminTicking.get_instance() otherwise.
            transitioncb:
                [optional] callback that is called at each transition, signature `def transitioncb(statemachine,blackboard,source,outcome)->outcome`
            statecb:
                [optional] callback that is called at each, signature `default_statecb(statemachine,blackboard,state)`

        warning:            
            TODO: name of output topic needs to be changed.
        """
        super().__init__(name,outcomes=[SUCCEED, ABORT,TIMEOUT],transitioncb=transitioncb,statecb=statecb)

        #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues:
        # I am not so sure that the transition will fail if inappropriate, only when there is an error for an appropriate transition.
        self.add_state(LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node),
                transitions={SUCCEED: "CLEANUP_ETASL",
                            ABORT: "CLEANUP_ETASL",
                            TIMEOUT: ABORT}) 
        
        #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues
        self.add_state(LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node),
                transitions={SUCCEED: "PARAMETER_CONFIG",ABORT: "PARAMETER_CONFIG"}) 

        self.add_state(SetTaskParameters( "PARAMETER_CONFIG",task_name, srv_name, cb, timeout, node ),
                       transitions={
                           SUCCEED: "ROBOT_SPECIFICATION"
                       })
        self.add_state(ReadRobotSpecification("ROBOT_SPECIFICATION",task_name,srv_name,timeout,node),
                transitions={SUCCEED: "TASK_SPECIFICATION"})

        self.add_state(ReadTaskSpecification("TASK_SPECIFICATION",task_name,srv_name,timeout,node),
                transitions={SUCCEED: "CONFIG_ETASL"})

        self.add_state(LifeCycle("CONFIG_ETASL",srv_name,Transition.CONFIGURE,timeout,node),
                transitions={SUCCEED: "ACTIVATE_ETASL"})

        self.add_state(LifeCycle("ACTIVATE_ETASL",srv_name,Transition.ACTIVATE,timeout,node),
                transitions={SUCCEED: "EXECUTING"})

        # executes until one returns SUCCEED,  eTaSLOutput only returns TICKING
        self.add_state(
            ConcurrentFallback("EXECUTING",[
                eTaSLEvent("check_event","/fsm/events",node=node,mapping={"e_finished@etasl_node":(1,SUCCEED)}),
                eTaSLOutput("output","/my_topic",bb_location=["output",name], node=node)
            ]),
            transitions={SUCCEED:SUCCEED}
        )        
