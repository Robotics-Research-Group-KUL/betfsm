# betfsm_ros.py
#
# Copyright (C) Erwin Aertbeliën, 2024
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

import re
from abc import abstractmethod
from typing import Dict, List, Union, Callable,Type, TypeAlias, Iterable, Optional
from collections import deque
from dataclasses import dataclass
from threading import Lock
import time

from rclpy.node import Node
import ament_index_python as aip

from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.time import Duration
from rclpy.action import ActionClient

from betfsm import (
    SUCCEED,ABORT,TIMEOUT,CANCEL, TICKING,
    add_logger_category, get_logger, 
    Blackboard,TickingState, TickingStateMachine, Generator,GeneratorWithState
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
                 node:Node = None, always_succeed=False) -> None:
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
            get_logger("service").info(f"Not get_client found in node creating service client for {self.srv_name} ({self.srv_type.__name__})")
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
                    get_logger().error(f"could not find service {self.srv_type.__name__} from {self.srv_name} in time")
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
                    get_logger().error(f"service {self.srv_type} from {self.srv_name} did not answer in time")
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
                 node:Node = None) -> None:
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
                name:str,
                srv_name:str = "/etasl_node", 
                transition: Transition=Transition.ACTIVATE, 
                timeout:Duration = Duration(seconds=1.0), 
                node:Node = None,
                always_succeed = False):
        """
        Parameters:
            name:
                instance name of the lifecycle action
            srv_name:
                name of the node whose lifecycle to control
            transition:
                indicates which transition
            timeout: 
                duration that indicates the timeout, 0 is forever
            node:
                if None, singleton BeTFSMNode.get_instance() will be used.
            always_succeed:
                if True, will always return SUCCEED or TICKING
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        outcomes = [SUCCEED,ABORT] # TIMEOUT added by ServiceClient, TICKING added by TickingState
        super().__init__(name,srv_name=srv_name+"/change_state",srv_type=ChangeState,outcomes=outcomes,timeout=timeout,node=node,always_succeed=always_succeed)
        self.transition = transition
        self.node_name = srv_name

    def fill_in_request(self, blackboard: Blackboard,request) -> None:
        #get_logger().info(f"Set lifecycle of {self.node_name} to {self.name}")
        request.transition.id = self.transition.value
        if self.transition.value == Transition.CONFIGURE.value:
            request.transition.label='configure'
        elif self.transition.value == Transition.CLEANUP.value:
            request.transition.label='cleanup'
        elif self.transition.value == Transition.ACTIVATE.value:
            request.transition.label='activate'
        elif self.transition.value == Transition.DEACTIVATE.value:
            request.transition.label='deactivate'
        else:
            request.transition.label='shutdown'
        return request
    
    def process_result(self, blackboard: Blackboard, result) -> str:
        if result.success:
            return SUCCEED
        else:
            return ABORT



@dataclass
class QueuedEvent:
    name: str
    timestamp: float


class TopicEventReceiver:
    """
    !!! warning
        Preliminary, untested version

        old name was: EventQueueSubscriber

    Thread-safe event queue for recieving ROS2 String messages

    Messages are received in a queue with limited size
    
    Functions for polling the queue, possibly with a max_age parameters
    to avoid stale events.

    No priorities. Multithread save.


    """
    _instances = {}

    @classmethod
    def get_instance( cls,  node: Node,  topic_name: str, queue_size: int = 10  ):
        """
        One singleton queue per topic.

        Parameters:
            node:
                ROS2 node
            topic_name:
                name of the topic to subscribe to
            queue_size:
                size of the queue (related to maximum concurrent events, i.e. sample time in relation
                to the events generated)
        Returns:
            Singleton instance of EventQueueSubscriber.
        """
        if topic_name not in cls._instances:
            cls._instances[topic_name] = cls(
                node=node,
                topic_name=topic_name,
                queue_size=queue_size
            )
        return cls._instances[topic_name]

    def __init__( self, node: Node,  topic_name: str,  queue_size: int):

        self.node = node
        self.topic_name = topic_name
        self.queue = deque(maxlen=queue_size)
        self.lock = Lock()
        qos = QoSProfile(
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.VOLATILE
        )
        self.subscription = node.create_subscription( String,  topic_name, self._callback, qos  )
        self.node.get_logger().info( f"TopicEventReceiver attached to {topic_name}" )

    def _callback(self, msg: String):
        event = QueuedEvent( name=msg.data, timestamp=time.monotonic() )
        with self.lock:
            self.queue.append(event)

    def has_event( self, target_strings: Iterable[str]  ) -> bool:
        """
        Looks for an event in the queue that matches one of the target_strings.
        Reads from the oldest to the newest element in the queueu.
        Returns the matching event and None if no event matches.
        It **does not** consume the matching event.
        """
        targets = set(target_strings)
        with self.lock:
            for i, event in enumerate(self.queue):
                if (   event.name in targets ):
                    matched = event.name
                    return matched
        
    def has_recent_event( self, target_strings: Iterable[str] ,  max_age_seconds: float  ) -> bool:
        """ 
        Looks for an event in the queue that matches one of the target_strings.
        Reads from the oldest to the newest element in the queueu.
        Returns the matching event and None if no event matches.
        It only returns events that are at most max_age_seconds old.  
        This can be useful to avoid stale events.
        It **does not** consume the matching event.
        """        
        now = time.monotonic()
        targets = set(target_strings)
        with self.lock:
            for i, event in enumerate(self.queue):
                age = now - event.timestamp
                if (   event.name in targets  and age <= max_age_seconds  ):
                    matched = event.name
                    return matched
            
    def poll_for( self, target_strings: Iterable[str] ) -> Optional[str]:
        """ 
        Looks for an event in the queue that matches one of the target_strings.
        Reads from the oldest to the newest element in the queueu.
        Returns the matching event and None if no event matches.
        It consumes the matching event.
        """
        targets = set(target_strings)
        with self.lock:
            for i, event in enumerate(self.queue):
                if event.name in targets:
                    matched = event.name
                    del self.queue[i]
                    return matched
        return None

    def poll_recent_for(  self,  target_strings: Iterable[str],  max_age_seconds: float ) -> Optional[str]:
        """ 
        Looks for an event in the queue that matches one of the target_strings.
        Reads from the oldest to the newest element in the queueu.
        Returns the matching event and None if no event matches.
        It only returns events that are at most max_age_seconds old.  
        This can be useful to avoid stale events.        
        It consumes the matching event.
        """
        now = time.monotonic()
        targets = set(target_strings)
        with self.lock:
            for i, event in enumerate(self.queue):
                age = now - event.timestamp
                if (   event.name in targets  and age <= max_age_seconds  ):
                    matched = event.name
                    del self.queue[i]
                    return matched
        return None

    def clear(self):
        """
        Clears the qeueue
        """
        with self.lock:
            self.queue.clear()

    def size(self) -> int:
        """
        Size of the queue
        """
        with self.lock:
            return len(self.queue)

    def log_queue(self, max_items: int = 20):
        """
        Logs a snapshot of queue.
        """
        with self.lock:
            def format_queue(q):
                return [ f"{e.name} (age={time.monotonic() - e.timestamp:.3f}s)"  for e in list(q)[:max_items]  ]
            msgs = format_queue(self.queue)
        self.node.get_logger().info(
            "=== Event Queue Snapshot ===\n"
            + "\n".join(msgs) +
            "\n============================"
        )

    def set_minimum_queue_size(self,sz : int):
        """
        sets the minimum queue size
        """
        with self.lock:
            if self.queue.maxlen < sz:
                new = deque(self.queue,maxlen=sz)
                self.queue = new



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

