# yasmin_ticking_ros.py
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

from .yasmin_ticking import *
from .tickingstatemachine import *

from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode
from lifecycle_msgs.srv import ChangeState

import re
import ament_index_python as aip

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from abc import ABC, abstractmethod

import heapq


from std_msgs.msg import String

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
    def __init__(self,timeout: Duration = Duration(seconds=1.0), node : Node = None):
        """
        TimedWait waits for a given time and then returns succeed.

        Parameters:
            timeout:
                duration to wait.
            node:
                Node to use for clock, 

        Returns:
            will return TICKING until timeout is passed after which it returns SUCCEED
        """
        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = None
        outcomes = [SUCCEED,ABORT]
        super().__init__("TimedWait",outcomes)
        self.clock = self.node.get_clock()
        self.log   = self.node.get_logger()
        self.timeout = timeout
    
    def co_execute(self,blackboard):     
        self.log.info(f"{self.name} : waiting for {self.timeout}")
        starttime = self.clock.now()
        while not self.clock.now() - starttime > self.timeout:
            yield TICKING
        self.log.info(f"{self.name} : finished waiting")
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
            maxcount: 
                maximum of iterations, if maxcount==0, repeat until SUCCEED is returned.   
            timeout: 
                underlying state is triggered every `timeout` duration
            state:
                underlying state
            node: 
                ROS2 node, if None, YasminNode.get_instance() is used

        Note:
            if the underlying state returns later than `timeout` with a non-ticking outcome, an exception will be raised 
            and abort is called.
        """
        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = node
        super().__init__("TimedRepeat",[SUCCEED],state)
        self.maxcount = maxcount
        self.timeout = timeout
        self.log  = self.node.get_logger()
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
    """
    def __init__(
            self,
            timeout: Duration,
            state: TickingState,
            node : Node = None  
        ):
        """
        Timeout passes through the outcomes of the underlying state, finishes if the outcome is not Ticking,
        and returns TIMEOUT when `timeout` duration is reached during execution.
        
        Parameters:
            timeout: 
                underlying state is triggered every `timeout` duration
            state:
                underlying state
            node: 
                ROS2 node, if None, YasminNode.get_instance() is used 

        Warning:
            assumes that the underlying state sufficiently yields TICKING!
            Don't use this if the underlying state completely blocks!
        """
        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = node        
        super().__init__("Timeout",[TIMEOUT],state)
        self.timeout = timeout
        self.log  = self.node.get_logger()
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
                 node:Node = None) -> None:
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
                node, if None, YasminNode.get_instance() will be used.
        """
        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = None        
        outcomes.append(TIMEOUT) #TickingState will add TICKING
        super().__init__(name,outcomes)        
        self.clock     = self.node.get_clock()  
        self.srv_type  = srv_type
        self.srv_name  = srv_name
        self.srvclient = self.node.create_client(srv_type,srv_name)
        self.request   = srv_type.Request()
        self.timeout   = timeout

    def co_execute(self,blackboard:Blackboard):
        get_logger("service").info(f"calling ROS2 service {self.srv_name} ({self.srv_type.__name__})")
        starttime = self.clock.now()
        while not self.srvclient.service_is_ready():
            if self.timeout!=Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger().error(f"could not find service {self.srv_type.__name__} from {self.srv_name} in time")
                    yield TIMEOUT
            yield TICKING
        self.request = self.fill_in_request(blackboard,self.request)
        future=self.srvclient.call_async(self.request)
        while not future.done():
            if self.timeout!=Duration():
                if self.clock.now() - starttime > self.timeout:
                    get_logger().error(f"service {self.srv_type} from {self.srv_name.__name__} did not answer in time")
                    yield TIMEOUT            
            yield TICKING
        result = future.result()
        get_logger("service").info(f"received results from {self.srv_name}({self.srv_type.__name__})")
        yield self.process_result(blackboard, result)        


    def fill_in_request(self,blackboard:Blackboard, request):
        """
        fills in the self.req object (of the type srv_type.Request) with
        the appropriate parameters of the service call

        Parameters:
            blackboard: 
                blackboard to be used
            request (srv_type.Request) : 
                request to be filled in

        Returns:
            request(srv_type.Request):
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



    



class EventTopicListener(Listener):
    """
    Following queuing policy:
        - either warn if max queue is reached or silently forget the oldest
        - filtered using the available transitions in state and state machine, and take the oldest one.

    """
    def __init__(
            self,
            topic: str,
            topic_type:Type,
            outcomes: List[str],
            queue:Queue=QueueFIFO(),
            msg_queue: int = 30,
            node: Node = None,
        ) -> None:
        if node==None:
            self.node = YasminNode.get_instance()
        else:
            self.node = node
        super().__init__(outcomes,queue=queue)
        self.topic_type = topic_type
        self.topic = topic
        self.msg_queue = msg_queue
        self.count = 0

        # qos_profile = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST, #Keeps the last msgs received in case buffer is fulll
        #     depth=msg_queue, #Buffer size
        #     reliability=QoSReliabilityPolicy.RELIABLE, #Uses TCP for reliability instead of UDP
        #     durability=QoSDurabilityPolicy.VOLATILE #Volatile, may not use first msgs if subscribed late (will not happen in this context)
        # )
        # self.subscription = self.node.create_subscription(
        #     topic_type,topic,self.__callback,qos_profile)
        self.subscription = self.node.create_subscription(
             self.topic_type,self.topic,self.__callback,self.msg_queue)
        self.monitoring=False        
        
    def __callback(self,msg) -> None:
        if self.monitoring:
            priority, outcome = self.process_message(msg)
            self.queue.push_outcome(priority, self.count,outcome,msg)
            self.count = self.count + 1

    
    @abstractmethod
    def process_message(self,msg) -> tuple[int,str]:
        """
        is called when a message on the topic is received. Processes the message into
        an (outcome, priority) tuple
        """
        raise NotImplementedError("set_payload abstract method is not implemented by subclass")
    
    @abstractmethod
    def set_payload(self, blackboard: Blackboard, msg):
        """
        is called when the outcome is returned.
        """
        pass

    def start(self):
        self.queue.clear()
        self.monitoring=True

    def stop(self):
        self.monitoring=False





# configure
# shutdown
# activate
# cleanup
# deactivate


# states:
# - unconfigured
# - inactive
# - active
# - finalized

from enum import Enum

class Transition(Enum):
    CONFIGURE=1
    CLEANUP=2
    ACTIVATE=3
    DEACTIVATE=4
    UNCONFIGURED_SHUTDOWN  = 5
    INACTIVE_SHUTDOWN = 6
    ACTIVE_SHUTDOWN = 7    

class LifeCycle(ServiceClient):
    """
    ROS2 lifecycle (simplified):
    ```mermaid
        stateDiagram-v2
            direction LR
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white

            
            [*] --> unconfigured
            unconfigured --> inactive : CONFIGURE
            inactive --> active : ACTIVATE
            inactive --> unconfigured : CLEANUP
            inactive --> finalized : INACTIVE_SHUTDOWN
            active --> inactive : DEACTIVATE
            active --> finalized : ACTIVE_SHUTDOWN
            unconfigured --> finalized : UNCONFIGURED_SHUTDOWN
    ```

    ```python
        class Transition(Enum):
            CONFIGURE              = 1
            CLEANUP                = 2
            ACTIVATE               = 3
            DEACTIVATE             = 4
            UNCONFIGURED_SHUTDOWN  = 5
            INACTIVE_SHUTDOWN      = 6
            ACTIVE_SHUTDOWN        = 7    
    ```
    A ROS 2 node can take a while before reaching the desired state.  These transitional states are
    not depicted here. Documentation of the full state machine can be found [here](https://design.ros2.org/articles/node_lifecycle.html)
    """
    def __init__(self, 
                srv_name:str = "/etasl_node", 
                transition: Transition=Transition.ACTIVATE, 
                timeout:Duration = Duration(seconds=1.0), 
                node:Node = None):
        """
        Parameters:
            srv_name:
                name of the node whose lifecycle to control
            transition:
                indicates which transition
            timeout: 
                duration that indicates the timeout, 0 is forever
            node:
                if None, singleton YasminNode.get_instance() will be used.
        """
        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = None
        outcomes = [SUCCEED,ABORT] # TIMEOUT added by ServiceClient, TICKING added by TickingState
        super().__init__(transition.name,srv_name+"/change_state",ChangeState,outcomes,timeout,node)
        self.transition = transition
        self.node_name = srv_name

    def fill_in_request(self, blackboard: Blackboard,request) -> None:
        get_logger().info(f"Set lifecycle of {self.node_name} to {self.name}")
        request.transition.id = self.transition.value
        return request
    
    def process_result(self, blackboard: Blackboard, result) -> str:
        if result.success:
            return SUCCEED
        else:
            return ABORT





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

import os    

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
