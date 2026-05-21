from betfsm.events import Condition
from betfsm_ros.betfsm_ros import  BeTFSMNode, Node
import time
from typing import Dict, List, Union, Callable,Type, TypeAlias,Iterable,Optional
from dataclasses import dataclass
from collections import deque

from threading import Lock
from std_msgs.msg import String
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


#####################################################################
#                   Event Receivers
#####################################################################

@dataclass
class QueuedEvent:
    name: str
    timestamp: float


class TopicEventReceiver:
    """
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
                node,
                topic_name,
                queue_size
            )
            instance = cls._instances[topic_name]
        else:
            instance = cls._instances[topic_name]
            with instance.lock:
                if instance.queue.maxlen < queue_size:
                    new = deque(instance.queue,maxlen=queue_size)
                    instance.queue = new
        return instance




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


#####################################################################
#                   Conditions / Triggers / Events
#####################################################################

class TopicEvent_Condition(Condition):
    def __init__(self,node=None,topic_name="crospi_node/my_output", queue_size=10, max_age=0.5, consume=True):   
        """
        To be used as a condition for EventOutcome, EventConcurrent, EventSequential        
        Parameters:
            node:
                ROS2 node
                
            topic_name:
                name of the topic to subscribe to

            queue_size:
                size of the queue related to maximum concurrent events, i.e. sample time in relation
                to the events generated. Because the underlying receiver is a singleton, the maximum
                queue size of everybody who requested an instance of the receiver is taken.

            max_age:
                maximum age of the events that still will be received.

            consume:
                consumes received events. in some special cases where different event checkers check the same
                event, only one will consume and the other will observe.
                
        """
        if node==None:
            node = BeTFSMNode.get_instance()
        self.node           = node
        self.topic_name     = topic_name
        self.queue_size     = queue_size            
        self.max_age        = max_age
        self.consume        = consume
        self.recv = TopicEventReceiver.get_instance(node,topic_name,queue_size)            

    def poll(self,bb,events):
        if self.consume:
            return self.recv.poll_recent_for(events, self.max_age)
        else:
            return self.recv.has_recent_event(events, self.max_age)
        
    def peek(self,bb,events):
        return self.recv.has_recent_event(events, self.max_age)
    
    def reset(self,bb,events):
        pass

    def __str__(self) -> str:
        return f"TopicEvent(node, {self.topic_name}, {self.queue_size}, {self.max_age}, {self.consume} )"


