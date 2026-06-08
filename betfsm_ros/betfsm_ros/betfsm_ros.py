# betfsm_ros.py
#
# region Copyright (C) Erwin Aertbeliën, 2024
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

import re
from abc import abstractmethod
from typing import Dict, List, Union, Callable,Type, TypeAlias, Iterable, Optional
import os    
import time

from rclpy.node import Node
import ament_index_python as aip
from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState,GetState
#from lifecycle_msgs.msg import Transition
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Duration
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from rclpy.time import Duration,Time
import numpy as np


from dataclasses import dataclass
from betfsm import (
    SUCCEED,ABORT,TIMEOUT,CANCEL, TICKING,
    add_logger_category, get_logger, 
    Blackboard,TickingState, TickingStateMachine, Generator,GeneratorWithState,
    ConfigCallback,get_path
)
from .betfsm_node import BeTFSMNode


add_logger_category("service")
add_logger_category("action")


class TimedWait(Generator):
    """Node that waits for a given time and then returns succeed
    ```mermaid
    stateDiagram-v2
        direction LR
        classDef greenClass  fill:darkgreen,color:white
        classDef yellowClass  fill:yellow,color:black
        classDef redClass  fill:darkred,color:white

        state "TimedWait (timeout )" as TimedWait
        [*] --> TimedWait
        TimedWait --> SUCCEED : timeout reached
        SUCCEED --> [*]
        TimedWait --> TICKING : ticking
        TICKING --> [*]
            
    
        class SUCCEED greenClass
        class TICKING yellowClass
        
    ```
    """
    def __init__(self,name:str,timeout: Duration = Duration(seconds=1.0), node : Node = None):
        """
        TimedWait waits for a given time and then returns succeed.

        Parameters:
            name:
                instance name
            timeout:
                duration to wait.
            node:
                Node to use for clock, 

        will return TICKING until timeout is passed after which it returns SUCCEED
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        outcomes = [SUCCEED,ABORT]
        super().__init__(name,outcomes)
        self.clock = self.node.get_clock()
        self.timeout = timeout
    
    def co_execute(self,blackboard):     
        get_logger().info(f"{self.name} : waiting for {self.timeout}")
        starttime = self.clock.now()
        while not self.clock.now() - starttime > self.timeout:
            yield TICKING
        get_logger().info(f"{self.name} : finished waiting")
        yield SUCCEED


class TimedRepeat(GeneratorWithState):
    """
    Repeats an underlying state for a given number of times and a given time interval.
    ```mermaid
        stateDiagram-v2
            direction LR
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white


            state TimedRepeat {
                direction TB
                [*] --> my_state
                state "State" as my_state
                state "Waiting" as waiting
                my_state --> waiting : succeed
                waiting --> my_state : timeout
            }
            [*] --> TimedRepeat
            my_state --> SUCCEED : #succeed > maxcount
            my_state --> OTHER   : other outcome
            my_state --> TICKING : ticking
            waiting  --> TICKING : ticking
            waiting --> ABORT : time > timeout
            class SUCCEED successClass
            class OTHER otherClass
            class TICKING tickingClass
            class ABORT abortClass
    ``` 
    """    
    def __init__(
            self,
            name:str,
            maxcount:int, 
            timeout: Duration,
            state: TickingState,
            node : Node = None  
        ):
        """
        TimedRepeat repeats the underlying state each `timeout` duration, until either the specified maxcount 
        iterations is reached or the underlying state returns anything else besides TICKING or SUCCEED.
        TimedRepeat returns TICKING or finishes with SUCCEED if maxcount is reached, or another outcome if 
        such outcome  is returned by the underlying state.

        Parameters:
            name:
                instance name
            maxcount: 
                maximum of iterations, if maxcount==0, repeat until SUCCEED is returned.   
            timeout: 
                underlying state is triggered every `timeout` duration
            state:
                underlying state
            node: 
                ROS2 node, if None, BeTFSMNode.get_instance() is used

        Note:
            if the underlying state returns later than `timeout` with a non-ticking outcome, an exception will be raised 
            and abort is called.
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        super().__init__(name,[SUCCEED],state)
        self.maxcount = maxcount
        self.timeout = timeout
        self.clock = self.node.get_clock()
    
    def co_execute(self,blackboard):        
        starttime = self.clock.now()
        looptime = starttime + self.timeout
        # for c in range(self.maxcount):
        count = 0
        while (self.maxcount==0) or (count < self.maxcount):
            # execute underlying state while ticking if necessary
            outcome = self.state(blackboard)
            while outcome==TICKING:
                yield TICKING
                outcome = self.state(blackboard)
            if outcome!=SUCCEED:
                yield outcome
            # check if time exceeded
            current_time = self.clock.now()
            if current_time > looptime:
                raise Exception("Loop time exceeded by underlying state")
            # tick until next timeout interval            
            while current_time < looptime:
                yield TICKING
                current_time = self.clock.now()
            looptime = looptime + self.timeout
            count    = count + 1
        yield SUCCEED
    

class Timeout(GeneratorWithState):
    """
    Timeout
    
    warning:
        obsolete, EventOutcome with Timeout_Condition recommended.
    """
    def __init__(
            self,
            name:str,
            timeout: Duration,
            state: TickingState,
            node : Node = None  
        ):
        """
        Timeout executes the underlying state at long as its outcome is TICKING. It finishes when 
        the outcome is not ticking and returns this outcome.  It also finishes when the given 
        duration is exceeded and returns TIMEOUT.
        
        Parameters:
            name:
                instance name
            timeout: 
                underlying state is triggered every `timeout` duration
            state:
                underlying state
            node: 
                ROS2 node, if None, BeTFSMNode.get_instance() is used 

        Warning:
            assumes that the underlying state sufficiently yields TICKING!
            Don't use this if the underlying state completely blocks!
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node        
        super().__init__("Timeout",[TIMEOUT],state)
        self.timeout = timeout
        self.clock = self.node.get_clock()        

    def co_execute(self,blackboard):        
        starttime = self.clock.now()
        timeout = starttime + self.timeout
        while (self.clock.now() <= timeout):
            outcome = self.state(blackboard)
            yield outcome
        yield TIMEOUT


class ServiceClient(Generator):
    """
    Creates a TickingState that calls a ROS2 service and generates an outcome when the service returns back.
    While waiting, it continues to tick.
    The methods `fill_in_request` and `process_results` should be overriden. Outcome is whatever `process_result`
    returns (should be listed in `outcomes` argument to constructor)

    ```mermaid
        stateDiagram-v2
            direction LR
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white


            state ServiceClient {
                direction TB
                [*] --> waiting
                state "Waiting for service" as waiting
                state "fill_in_request() <br> calling service" as calling
                state "Waiting for result<br>process_results()" as result
                waiting --> calling : ready
                calling --> result : request send
            }
            [*] --> ServiceClient
            waiting --> TIMEOUT : time exceeded
            waiting --> TICKING  
            result --> TICKING : ticking
            result  --> TIMEOUT : time exceeded
            result  --> OUTCOME : result processed
            
            class SUCCEED successClass+
            class OUTCOME otherClass
            class TICKING tickingClass
            class TIMEOUT abortClass
    ```

    """
    def __init__(self, name:str, srv_name:str, srv_type: Type, outcomes:List[str], timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None, always_succeed:bool=False) -> None:
        """
        Creates a TickingState that calls a service and generates an outcome when the service returns back.
        While waiting, it continues to tick.

        Parameters:
            name:
                name of the state
            srv_name:
                name of the service
            srv_type:
                type of the service
            outcomes:
                outcomes to be expected (TIMEOUT and TICKING will be added)
            timeout:
                maximum time for contacting service and processing and retrieving request.
                (special value: Duration(): ad infinitum)
            node:
                node, if None, BeTFSMNode.get_instance() will be used.
            always_succeed:
                Do your best, but always return SUCCEED or TICKING
        """
        if not( isinstance(name,str) and isinstance(srv_name,str) and isinstance(outcomes,list) ):
            raise ValueError("Error in argument types of constructor")
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        if not always_succeed:
            outcomes.append(TIMEOUT) #TickingState will add TICKING
        outcomes.append(SUCCEED) #TickingState will add TICKING
        super().__init__(name,outcomes)        
        self.clock     = self.node.get_clock()  
        self.srv_type  = srv_type
        self.srv_name  = srv_name
        # check if self.node has a get_client method
        if not hasattr(self.node, 'get_client'):
            get_logger("service").info(f"No get_client found in node creating service client for {self.srv_name} ({self.srv_type.__name__}), creating a service client")
            self.srvclient = self.node.create_client(srv_type,srv_name)
        else:
            self.srvclient = self.node.get_client(srv_type,srv_name)

        self.request   = srv_type.Request()
        self.timeout   = timeout
        self.always_succeed = always_succeed

    def co_execute(self,blackboard:Blackboard):
        get_logger("service").info(f"calling ROS2 service {self.srv_name} ({self.srv_type.__name__})")
        starttime = self.clock.now()
        while not self.srvclient.service_is_ready():
            if self.timeout!=Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger().warning(f"could not find service {self.srv_type.__name__} from {self.srv_name} in time")
                    if self.always_succeed:
                        yield SUCCEED
                    else:
                        yield TIMEOUT
            yield TICKING
        self.request = self.fill_in_request(blackboard,self.request)
        future=self.srvclient.call_async(self.request)
        while not future.done():
            if self.timeout!=Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger().warning(f"service {self.srv_type} from {self.srv_name} did not answer in time")
                    if self.always_succeed:
                        yield SUCCEED
                    else:
                        yield TIMEOUT           
            yield TICKING
        result = future.result()
        
        outcome = self.process_result(blackboard, result)
        get_logger("service").info(f"received results from {self.srv_name}({self.srv_type.__name__}) with outcome {outcome}")
        if self.always_succeed:
            yield SUCCEED
        else:
            yield outcome
        #yield self.process_result(blackboard, result)        


    def fill_in_request(self,blackboard:Blackboard, request:Type) -> Type:
        """
        fills in the self.req object (of the type srv_type.Request) with
        the appropriate parameters of the service call

        Parameters:
            blackboard: 
                blackboard to be used
            request: 
                request to be filled in

        Returns:
            request that was filled in
        """
        return request

    def process_result(self,blackboard:Blackboard, result) -> str:
        """
        gets the result and puts it in the blackboard (if needed) and
        returns an outcome

        Parameters:
            blackboard:
                blackboard to be used
            result(srv_type.Result):  
                result returned by the service

        Returns:
            outcome : str
                the outcome to give back (should be final outcome, i.e. TICKING not allowed)
        """
        return SUCCEED


class ActionClientBTFSM(Generator):

    def __init__(self, name:str, action_name:str, action_type: Type, outcomes:List[str], timeout:Duration = Duration(seconds=1.0), 
                 node:Node|None = None) -> None:
        """
        Creates a TickingState that calls an action and generates an outcome when the action returns back.
        While waiting, it gets the response of the action to the blackboard and returns TICKING.

        Parameters:
            name:
                name of the state
            action_name:
                name of the service
            action_type:
                type of the service
            outcomes:
                outcomes to be expected (TIMEOUT and TICKING will be added)
            timeout:
                maximum time for contacting service and processing and retrieving request.
                (special value: Duration(): ad infinitum)
            node:
                node, if None, BeTFSMNode.get_instance() will be used.
        """
        # import time
        # start_time = time.time()
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        outcomes.extend([TIMEOUT, SUCCEED])
        super().__init__(name, outcomes)
        self.clock          = self.node.get_clock()  
        self.action_type    = action_type
        self.action_name    = action_name
        if not hasattr(self.node, 'get_action'):
            self.client     = ActionClient(self.node, action_type, action_name)
        else:
            self.client = self.node.get_action(action_type, action_name)
        # end_time = time.time()
        # print(f"----JJJJJJJJJJJJJJJJJJJJ -----Time to create ActionClientBTFSM super: {end_time - start_time} seconds")
        self.timeout        = timeout
        self.goal_handle    = None
        self.result_future  = None
        self.send_goal_future = None

    def co_execute(self, blackboard: Blackboard):
        get_logger("action").info(f"Calling ROS2 action {self.action_name} ({self.action_type.__name__})")
        starttime = self.clock.now()
        # wait for the action server to be available
        while not self.client.wait_for_server(timeout_sec=self.timeout.nanoseconds / 1e9):
            if self.timeout != Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger("action").error(f"Could not find action server {self.action_name} in time")
                    yield TIMEOUT
            yield TICKING

        # Create the goal message and fill it in
        goal_msg = self.fill_in_goal(blackboard)
        self.send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        while not self.send_goal_future.done():
            if self.timeout != Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger("action").error(f"Goal to {self.action_name} timed out while sending")
                    yield TIMEOUT
            yield TICKING

        # Check if the goal was accepted by the action server
        self.goal_handle = self.send_goal_future.result()
        if not self.goal_handle.accepted:
            get_logger("action").error(f"Goal rejected by {self.action_name}")
            yield TIMEOUT
            return

        # Wait for the result
        self.result_future = self.goal_handle.get_result_async()
        while not self.result_future.done():
            if self.timeout != Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger("action").error(f"Action {self.action_name} did not finish in time")
                    yield TIMEOUT
            yield TICKING

        result = self.result_future.result().result
        get_logger("action").info(f"Received result from {self.action_name}")
        yield self.process_result(blackboard, result)
    
    def fill_in_goal(self, blackboard: Blackboard) -> Type:
        """
        fills in the self.action_type.Goal instance with
        the goal parameters of the action call

        Parameters:
            blackboard: 
                blackboard to be used

        Returns:
            self.action_type.Goal:
                goal to be used for the action call
        """
        return self.action_type.Goal()

    def process_result(self, blackboard: Blackboard, result) -> str:
        """
        gets the result and puts it in the blackboard (if needed) and
        returns an outcome

        Parameters:
            blackboard:
                blackboard to be used
            result(action_type.Result):  
                result returned by the action

        Returns:
            outcome : str
                the outcome to give back (should be final outcome, i.e. TICKING not allowed)
        """
        return SUCCEED

    def feedback_callback(self, feedback_msg):
        """
        Callback for feedback messages from the action server

        Parameters:
            feedback_msg ():
                feedback message from the action
        """
        pass


from enum import Enum


class Transition(Enum):
    """Transitions of a ROS2 lifecycle.

        The states are: unconfigured; inactive; active; finalized
        See doc. LifeCycle for a state diagram.
    """
    CONFIGURE=1
    CLEANUP=2
    ACTIVATE=3
    DEACTIVATE=4
    UNCONFIGURED_SHUTDOWN  = 5
    INACTIVE_SHUTDOWN = 6
    ACTIVE_SHUTDOWN = 7    

# class LifeCycle(ServiceClient):
#     """
#     ROS2 lifecycle (simplified):
#     ```mermaid
#         stateDiagram-v2
#             direction LR
#             classDef successClass  fill:darkgreen,color:white
#             classDef tickingClass  fill:yellow,color:black
#             classDef otherClass  fill:darkorange,color:white
#             classDef abortClass  fill:darkred,color:white

            
#             [*] --> unconfigured
#             unconfigured --> inactive : CONFIGURE
#             inactive --> active : ACTIVATE
#             inactive --> unconfigured : CLEANUP
#             inactive --> finalized : INACTIVE_SHUTDOWN
#             active --> inactive : DEACTIVATE
#             active --> finalized : ACTIVE_SHUTDOWN
#             unconfigured --> finalized : UNCONFIGURED_SHUTDOWN
#     ```

#     ```python
#         class Transition(Enum):
#             CONFIGURE              = 1
#             CLEANUP                = 2
#             ACTIVATE               = 3
#             DEACTIVATE             = 4
#             UNCONFIGURED_SHUTDOWN  = 5
#             INACTIVE_SHUTDOWN      = 6
#             ACTIVE_SHUTDOWN        = 7    
#     ```
#     A ROS 2 node can take a while before reaching the desired state.  These transitional states are
#     not depicted here. Documentation of the full state machine can be found [here](https://design.ros2.org/articles/node_lifecycle.html)
#     """
#     def __init__(self, 
#                 name:str,
#                 srv_name:str = "/etasl_node", 
#                 transition: Transition=Transition.ACTIVATE, 
#                 timeout:Duration = Duration(seconds=1.0), 
#                 node:Node = None,
#                 always_succeed = False):
#         """
#         Parameters:
#             name:
#                 instance name of the lifecycle action
#             srv_name:
#                 name of the node whose lifecycle to control
#             transition:
#                 indicates which transition
#             timeout: 
#                 duration that indicates the timeout, 0 is forever
#             node:
#                 if None, singleton BeTFSMNode.get_instance() will be used.
#             always_succeed:
#                 if True, will always return SUCCEED or TICKING
#         """
#         if node is None:
#             self.node = BeTFSMNode.get_instance()
#         else:
#             self.node = node
#         outcomes = [SUCCEED,ABORT] # TIMEOUT added by ServiceClient, TICKING added by TickingState
#         super().__init__(name,srv_name=srv_name+"/change_state",srv_type=ChangeState,outcomes=outcomes,timeout=timeout,node=node,always_succeed=always_succeed)
#         self.transition = transition
#         self.node_name = srv_name
#     def fill_in_request(self, blackboard: Blackboard,request) -> None:
#         #get_logger().info(f"Set lifecycle of {self.node_name} to {self.name}")
#         request.transition.id = self.transition.value
#         if self.transition.value == Transition.CONFIGURE.value:
#             request.transition.label='configure'
#         elif self.transition.value == Transition.CLEANUP.value:
#             request.transition.label='cleanup'
#         elif self.transition.value == Transition.ACTIVATE.value:
#             request.transition.label='activate'
#         elif self.transition.value == Transition.DEACTIVATE.value:
#             request.transition.label='deactivate'
#         else:
#             request.transition.label='shutdown'
#         return request
#
#     def process_result(self, blackboard: Blackboard, result) -> str:
#         if result.success:
#             return SUCCEED
#         else:
#             return ABORT

class LifeCycleTransition(Generator):
    def __init__(self,  name:str, srv_node:str, transition:Transition, timeout:Duration=Duration(seconds=1.0), node:BeTFSMNode=None):
        """Performs a transition on a ROS2 managed node

        Parameters
        ----------
        name : str
            _description_
        srv_node : str
            node that offers the service, service will be e.g. "{srv_node}/get_state"
        transition:
            indicates which transition
        timeout : Duration, optional
            _description_, by default Duration(seconds=1.0)
        node : BeTFSMNode, optional
            BeTFSMNode, if not specified the singleton instance, by default None
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        super().__init__(name,[SUCCEED,TIMEOUT,CANCEL])
        self.srv_node = srv_node
        self.change_state_srv_name = f"{srv_node}/change_state"
        self.change_state_client = self.node.get_client(ChangeState,self.change_state_srv_name)
        self.timeout=timeout
        self.transition=transition

    def set_start_time(self):
        self.starttime = self.node.get_clock().now()

    def wait_with_timeout(self, description:str, cb):
        while not cb():
            if self.node.get_clock().now() - self.starttime > self.timeout:
                get_logger().warning(f"LifeCycleTransition {self.name}: timout occurred  in {description}")
                yield TIMEOUT
            yield TICKING
        return SUCCEED # ensures that yield from ends

    def co_execute(self, blackboard):
        self.set_start_time()
        try:
            # waiting for service is ready:
            yield from self.wait_with_timeout(
                f"{self.change_state_srv_name} service_is_ready",
                self.change_state_client.service_is_ready)
            
            req = ChangeState.Request()
            req.transition.id = self.transition.value 
            current_future = self.change_state_client.call_async( req)
            yield from self.wait_with_timeout( f"ChangeState request of '{self.srv_node}' with transition {self.transition.name} ", current_future.done)
            response = current_future.result()
            if response.success:
                get_logger("service").info(f"LifeCycleTransition: target {self.srv_node} performed transition {self.transition.name}"  )
                yield SUCCEED
                return 
            else:
                get_logger().error(f"LifeCycleTransition: Transition request to '{self.srv_node}' with transtion {self.transition.name} was rejected")
                yield CANCEL
                return 
        except Exception as e:
            get_logger().error( f"LifeCycleTransition: request to {self.srv_node} failed: {str(e)}")
            yield CANCEL
        return








class ResetLifeCycleState(Generator):
    def __init__(self,  name:str, srv_node:str, timeout:Duration=Duration(seconds=1.0), node:BeTFSMNode=None):
        """_summary_

        Parameters
        ----------
        name : str
            _description_
        srv_node : str
            node that offers the service, service will be e.g. "{srv_node}/get_state"
        timeout : Duration, optional
            _description_, by default Duration(seconds=1.0)
        node : BeTFSMNode, optional
            BeTFSMNode, if not specified the singleton instance, by default None
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        super().__init__(name,[SUCCEED,TIMEOUT,CANCEL])
        self.srv_node = srv_node
        self.get_state_srv_name = f"{srv_node}/get_state"
        self.change_state_srv_name = f"{srv_node}/change_state"
        self.get_state_client = self.node.get_client(GetState,self.get_state_srv_name)
        self.change_state_client = self.node.get_client(ChangeState,self.change_state_srv_name)
        self.timeout=timeout



    def set_start_time(self):
        self.starttime = self.node.get_clock().now()


    def wait_with_timeout(self, description:str, cb):
        while not cb():
            if self.node.get_clock().now() - self.starttime > self.timeout:
                get_logger().warning(f"ToLifeCycleState {self.name}: timout occurred  in {description}")
                yield TIMEOUT
            yield TICKING
        return SUCCEED # ensures that yield from ends

    def co_execute(self, blackboard):
        self.set_start_time()
        try:
            while True:
                # waiting for service is ready:
                yield from self.wait_with_timeout(
                    f"{self.get_state_srv_name}.service_is_ready",
                    self.get_state_client.service_is_ready)
                
                # wating for get_state()
                current_future = self.get_state_client.call_async( GetState.Request())
                yield from self.wait_with_timeout( f"GetState request to {self.srv_node}", current_future.done)
                response = current_future.result() 
                current_label = response.current_state.label 
                get_logger("service").info(f"ResetLifeCycleState: target {self.srv_node} is currently in state '{current_label}'")

                # deciding next transition:
                req = ChangeState.Request()
                lbl = ""
                if current_label=="unconfigured":
                    yield SUCCEED
                    break
                elif current_label=="active":
                    lbl="TRANSITION_DEACTIVATE"
                    req.transition.id = Transition.DEACTIVATE.value
                    current_future = self.change_state_client.call_async( req)
                elif current_label=="inactive":
                    lbl="TRANSITION_CLEANUP"
                    req.transition.id = Transition.CLEANUP.value
                    current_future = self.change_state_client.call_async( req)
                else:
                    get_logger().error(f"Cannot reset from state: {current_label}")
                    yield CANCEL
                    break 
                # processing respone of transition request:
                yield from self.wait_with_timeout( f"ChangeState request of '{self.srv_node}' with transition {lbl} ", current_future.done)
                response = current_future.result()
                if response.success:
                    continue 
                else:
                    get_logger().error(f"Transition request ({lbl}) rejected by {self.srv_node}")
                    yield CANCEL
                    break
        except Exception as e:
            get_logger().error( f"ResetLifeCycleState: transition servie {self.srv_node} failed: {str(e)}")
            yield CANCEL
        return






#################################################################
#   Markers
#################################################################


class MarkerPublisher(Generator):
    def __init__(self, name:str, 
                 getter:ConfigCallback,
                 marker:Marker|int|None=None,
                 frequency:float=10,
                 array_type: bool = True,   #an array type of marker such as LINE_STRIP or POINTS
                 marker_id:int=0,
                 frame_id:str="world",
                 lifetime_s:  int=0,
                 color : ColorRGBA = None,
                 scale: float = 1.0,
                 node:Node=None
                 ):
        """


        Parameters
        ----------
        name : str
            _description_
        getter : ConfigCallback 
           callback class that returns the location(s) of the marker 
        marker : Marker | int | None, optional
            _description_, by default None
        frequency : float, optional
            _description_, by default 10
        array_type : bool, optional
            _description_, by default True
        frame_id : str, optional
            _description_, by default "world"
        node : Node, optional
            _description_, by default None
        """
        if node is None:
            node = BeTFSMNode.get_instance()
        super().__init__(name,[SUCCEED])
        self.node         = node
        self.getter       = getter 
        if marker is None or marker == Marker.LINE_STRIP:
            marker = Marker()
            marker.type   = Marker.LINE_STRIP # Type of marker (SPHERE, CUBE, ARROW, CYLINDER, etc.)
            marker.action = Marker.ADD    # ADD or DELETE marker
            marker.scale.x = 0.003*scale  # Diameter along X [m]
            marker.scale.y = 0.003*scale  # Diameter along X [m]
            marker.scale.z = 0.003*scale  # Diameter along X [m]
            marker.color.r = 1.0  # from 0..1
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Opacity (1.0 = completely solid)
            marker.id      = 0
            marker.ns     = "betfsm"
            marker.lifetime = Duration(seconds=lifetime_s).to_msg()
            marker.header.frame_id = "world"
        if marker is None or marker == Marker.SPHERE_LIST:
            marker = Marker()
            marker.type   = Marker.SPHERE_LIST # Type of marker (SPHERE, CUBE, ARROW, CYLINDER, etc.)
            marker.action = Marker.ADD    # ADD or DELETE marker
            marker.scale.x = 0.02*scale  # Diameter along X [m]
            marker.scale.y = 0.02*scale  # Diameter along X [m]
            marker.scale.z = 0.02*scale  # Diameter along X [m]
            marker.color.r = 1.0  # from 0..1
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Opacity (1.0 = completely solid)
            marker.id      = 0
            marker.ns     = "betfsm"
            marker.lifetime = Duration(seconds=lifetime_s).to_msg()
            marker.header.frame_id = "world"
        if color is not None:
            marker.color = color
        self.marker       = marker
        if marker_id is not None:
            self.marker.id = marker_id
        if frame_id is not None:
            self.marker.header.frame_id = frame_id
        self.frequency    = frequency
        self.publisher_created = False
    #.doc("type of the marker: \n ARROW=0,CUBE=1,SPHERE=2,CYLINDER=3,LINE_STRIP=4,LINE_LIST=5,CUBE_LIST=6,SPHERE_LIST=7,\n POINTS=8,TEXT=9,MESH_RESOURCE=10 (see ROS doc)")
    
    def co_execute(self, blackboard):
        # this waits until the input for MarkerPublisher is available:
        # or do nothing (MarkerPublisher is probably not a critical task, so we continue)
        logged = False 
        while not self.getter.reset(self,blackboard):
            if not logged:
                get_logger("crospi").warning(f"MarkerPublisher '{self.name}' configuration error in callback (ConfigCallback)")
                logged=True
            self.publisher_created = False
            yield TICKING
        self.publisher_created = True
        get_logger("crospi").info(f"MarkerPublisher '{self.name}' started publishing at frequency {self.frequency} Hz")
        self.publisher = self.node.create_publisher(Marker, '/visualization_marker', 10)
        sample_time = 0
        now         = self.node.get_clock().now()
        previous    = now
        while True:
            now = self.node.get_clock().now()
            if (now-previous).nanoseconds >= sample_time:
                sample_time = 1E9 / self.frequency
                lst = self.getter(self,blackboard)
                self.marker.header.stamp = now.to_msg()
                self.marker.points = [ Point( x=r[0], y=r[1], z=r[2] ) for r in lst ] 
                self.publisher.publish(self.marker)
                previous = now
            yield TICKING

    def exit(self):
        # this is also called when its externally reset or naturally finishes by itself (don't forget to call super().exit())
        if self.publisher_created:
            self.node.destroy_subscription(self.publisher)
            get_logger("crospi").info(f"MarkerPublisher '{self.name}' stopped publishing")
        return super().exit()




#################################################################
#   TF2 listeners and broadcasting
#################################################################


@dataclass
class TFSpec:
    """ 
    semantics : v_{wrt_to} = T_{wrt_to}_{of} *  v_{of} 
    """
    wrt_to: str 
    of: str
    path  : str
    is_matrix: bool = False


class TF2Listener(Generator):
    def __init__(self, name:str, tf_list:List[TFSpec], desired_outcome=str,node:BeTFSMNode=None):
        # tf_list: (tgt,src,path)
        if not node:
            node = BeTFSMNode.get_instance()
        self.node = node
        super().__init__(name,outcomes=[desired_outcome])
        self.tf_buffer = node.get_transformListenerBuffer()
        self.tf_list   = tf_list
        self.desired_outcome = desired_outcome

    def co_execute(self, blackboard):
        default_quat = np.array([0,0,0,1,0,0,0],dtype=np.float64)
        default_mat  = np.eye(4,dtype=np.float64)
        tf_bb = []
        for spec in self.tf_list:
            # if does not exist we choose default==[], such that we can assign a list to it without
            # needing access to the parent dict, using [:]
            if spec.is_matrix:
                tf_bb.append( get_path(self,blackboard,spec.path,default=default_mat.copy(),force=True))
            else:
                tf_bb.append( get_path(self,blackboard,spec.path,default=default_quat.copy(),force=True))
        while True:
            for i,spec in enumerate(self.tf_list):
                T=self.tf_buffer.lookup_transform(spec.wrt_to,spec.of,Time())
                loc =  tf_bb[i]
                if spec.is_matrix:
                    loc[:3,:3] = spR.from_quat( [T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z], scalar_first=True).as_matrix()
                    loc[:3,3]  = [  T.transform.translation.x, T.transform.translation.y, T.transform.translation.z]
                else:
                    # this still accesses the blackboard due to [:]
                    loc[:] = [  T.transform.translation.x, T.transform.translation.y, T.transform.translation.z,
                                        T.transform.rotation.w, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z]
            if self.desired_outcome!=TICKING:
                break 
            yield self.desired_outcome
        yield self.desired_outcome

class TF2Broadcaster(Generator):
    def __init__(self, name:str, 
                 tf_list:List[TFSpec],
                 frequency=10,
                 node:BeTFSMNode=None
                 ):
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        super().__init__(name,[])
        self.tf_list       = tf_list 
        self.frequency     = frequency

    def co_execute(self, blackboard):
        self.tf_broadcaster     = self.node.get_transformBroadcaster() 
        self.tf = TransformStamped()
        tf_bb = []
        for spec in self.tf_list:
            tf_bb.append( get_path(self,blackboard,spec.path))
        ns_interval = int(1E9 / self.frequency)
        previous    = self.node.get_clock().now() - Duration(nanoseconds=ns_interval)
        warnings      = []
        while True:
            now = self.node.get_clock().now()
            if (now-previous).nanoseconds >= ns_interval:
                previous = now
                # do something
                for i,spec in enumerate(self.tf_list):
                    #[x,y,z,qw,qx,qy,qz] = self.get_tf(blackboard)
                    loc=tf_bb[i] 
                    if not isinstance(loc,np.ndarray):
                        if not 1 in warnings:
                            get_logger().warning(f"TF2Broadcaster: location in blackboard is not a numpy array: {loc}")
                            warnings.append(1)
                        continue
                    if spec.is_matrix:
                        if loc.shape != (4,4):
                            if not 2 in warnings:
                                get_logger().warning(f"TF2Broadcaster: location in blackboard has wrong shape {loc.shape}\n{loc}")
                                warnings.append(2)
                            continue
                        q=spR.from_matrix(loc[:3,:3]).as_quat(scalar_first=True)
                        p=loc[:3,3]
                        x,y,z,qw,qx,qy,qz = p[0],p[1],p[2], q[0], q[1],q[2],q[3]
                    else:
                        if loc.shape != (7,):
                            if not 3 in warnings:
                                get_logger().warning(f"TF2Broadcaster: location in blackboard has wrong shape {loc.shape}\n{loc}")
                                warnings.append(3)
                            continue
                        [x,y,z,qw,qx,qy,qz] = loc
                    self.tf.header.frame_id = spec.wrt_to
                    self.tf.child_frame_id  = spec.of
                    self.tf.header.stamp = now.to_msg()    
                    self.tf.transform.translation.x = float(x)
                    self.tf.transform.translation.y = float(y)
                    self.tf.transform.translation.z = float(z)
                    self.tf.transform.rotation.x = float(qx)
                    self.tf.transform.rotation.y = float(qy)
                    self.tf.transform.rotation.z = float(qz)
                    self.tf.transform.rotation.w = float(qw)
                    self.tf_broadcaster.sendTransform(self.tf)
            yield TICKING

    def exit(self):
        # this is also called when its externally reset or naturally finishes by itself
        return super().exit()







################################################################################
#       Utility functions:
################################################################################

def expand_package_ref( pth:str ) -> str:
    """
    expands all occurencies of $PACKAGENAME] to the ROS2 packages they refer to.

    Parameters:
      pth: string to expand
    """
    def lookup(m):
        pkg=m.group(1)
        try:
            pkg_pth = aip.get_package_share_directory(pkg)
        except:
            raise Exception( f"ament did not find package '{pkg}' while expanding '{pth}'")
        return pkg_pth
    return re.sub(r"\$\[(\w+)\]",lookup, pth)


def expand_env_ref( pth: str ) -> str:
    """
    expands all occurencies of $[ENVVAR] or $ENVVAR to the ROS2 packages they refer to.
    Parameters:
        pth: string to expand
    """
    def lookup(m):
        var=m.group(1)
        if var in os.environ:
            value = os.environ[var]
        else:
            raise Exception( f"not find environment variable '{var}' while expanding '{pth}'")
        return value
    pth= re.sub(r"\$\{(\w+)\}",lookup, pth)
    #pth= re.sub(r"\$(\w+)",lookup, pth)
    return pth


def expand_ref(pth: str) -> str:
    """
    Expands both package references and environment variables.
    
    Parameters:
        pth: string to expand
    """    
    return expand_package_ref(expand_env_ref(pth))

