import time, signal, math
from typing import Dict, List, Union, Callable,Type, TypeAlias,Iterable,Optional
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from threading import Lock
from .logger import get_logger
from .betfsm import (
    SUCCEED,ABORT,TIMEOUT, CANCEL,NO_EVENT,TICKING,
    TickingState,Generator, GeneratorWithList,AlwaysOutcome,
    get_path_value, set_path_value
)






#####################################################################
#                   Event Receivers
#####################################################################


class Ctrl_C_Receiver:    
    """
    Installs a ctrl-C handler that puts a value into the blackboard. 
    Should only be created using the get_instance class method.
    """
    _instance = {}

    @classmethod
    def get_instance( cls,  value:str="CTRL_C", repeated=math.inf  ):
        """
        One singleton queue per topic.

        Configures the system SIGINT (Ctrl+C) handler to interact with a blackboard, where
        a variable will set to given value if Ctrl-c is pressed

        Arguments:
            value (str):
                Value that will be set when ctrl-c is pressed.
            repeated (int, None): 
                number of times to repeat ctrl-c before KeyboardInterrupt is raised

        Returns:
            Singleton instance of EventQueueSubscriber.

        Example:
            To start monitoring for Ctrl+C.  If ctrl-c is pressed, at the given path in the blackboard,
            a `True` value will be written.

            ```
            Ctrl_C_Handler(blackboard,"/cancelation/ctrl_c",repeated=3) 
            ```            
        """
        cls._instance = cls(value,repeated)
        return cls._instance 
    
    def __init__(self, value:str="CTRL_C", repeated=math.inf):
        self.repeated   = repeated
        self.value      = value
        self.count      = 0
        self.buffer     = ""
        assert( isinstance(value,str)  )        
        signal.signal(signal.SIGINT, self._handler )

    def remove_handler(self):
         """ 
         go back to the default ctrl-C handler
         """
         signal.signal(signal.SIGINT, signal.default_int_handler)       

    def _handler(self,signum, frame):
        self.buffer = self.value
        self.count = self.count+1
        get_logger().info(f"Ctrl-C pressed {self.count} times, pressing {self.repeated} times will interrupt")            
        if self.count >= self.repeated:
            raise KeyboardInterrupt
    
    def has_event( self, target_strings: Iterable[str]  ) -> bool:
        """
        checks whether one of the target_strings is present in the blackboard.        
        """        
        if self.buffer in target_strings:
            return self.buffer
        else:
            return None

    def poll_for( self, target_strings: Iterable[str] ) -> Optional[str]:      
        """ 
        checks whether one of the target_strings is present in the blackboard and consumes 
        the event, i.e. sets the value in blackboard to 
        """          
        if self.buffer in target_strings:
            retval = self.buffer
            self.buffer = ""
            return retval
        else:
            return None



class HTTPEventReceiver:
    """
    """

    @dataclass
    class QueuedEvent:
        name: str
        timestamp: float


    _instances = {}

    @classmethod
    def get_instance( cls, channel_name: str, queue_size: int = 10  ):
        """
        One singleton queue per topic.

        Parameters:
            channel_name:
                name of the topic to subscribe to
            queue_size:
                size of the queue (related to maximum concurrent events, i.e. sample time in relation
                to the events generated)
        Returns:
            Singleton instance of EventQueueSubscriber.
        """        
        if channel_name not in cls._instances:
            cls._instances[channel_name] = cls(
                channel_name,
                queue_size=queue_size
            )
            instance = cls._instances[channel_name]
        else:
            instance = cls._instances[channel_name]
            with instance.lock:
                if instance.queue.maxlen < queue_size:
                    new = deque(instance.queue,maxlen=queue_size)
                    instance.queue = new
        return instance

    @classmethod
    def lookup_instance(cls, channel_name):
        return cls._instances.get(channel_name,None)

    def __init__( self, channel_name: str,  queue_size: int):
        self.channel_name = channel_name
        self.queue = deque(maxlen=queue_size)
        self.lock = Lock()

    def push(self, ev: str):
        event = HTTPEventReceiver.QueuedEvent( name=ev, timestamp=time.monotonic() )
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


#####################################################################
#                   Conditions / Triggers / Events
#####################################################################




class Condition(ABC):
    """
    Base class for Conditions.  By convention we call
    derived classes:
      - ...Trigger when it only triggers one succesful poll, i.e. it consumes
    the event.  
      -  ...Condition, a version that does **NOT** consume the event, i.e. the 
      next tick it will return again the event, unless something changed.
      - . ...Event, a version where the events are read from some queue
    """

    @abstractmethod
    def poll(self,bb,events) -> str:
        """ poll() consumes the event/trigger and returns it """
        raise NotImplementedError
    
    @abstractmethod
    def peek(self,bb,events) -> str:
        """ peek() never consumes and returns the event """
        raise NotImplementedError
    
    @abstractmethod
    def reset(self,bb,events):
        """ called at the beginning of EventOutcome, EventConcurrent, EventSequential """
        pass

    def __str__(self) -> str:
        return "Condition"    

    def __and__(self, other):     # operator &
        return AndCondition(self,other)

    def __or__(self, other):      # operator |
        return OrCondition(self,other)


class OrCondition(Condition):
    """
    Combines condition1 and condition2 using OR.

    Typically constructoed by applying the operator `|` to two objects.

    - Will not consume if the OR returns None.
    - Only one of the conditions is consumed.  
    - condition1 is consumed if it returns a string, i.e. condition1 has priority.
    """
    def __init__(self, cond1, cond2):
        self.cond1 = cond1
        self.cond2 = cond2

    def poll(self,bb,events) -> str:
        r1 = self.cond1.poll(bb,events)
        if r1:
            return r1
        return self.cond2.poll(bb,events)

    def peek(self,bb,events) -> str:
        r1 = self.cond1.peek(bb,events)
        if r1:
            return r1
        return self.cond2.peek(bb,events)
    
    def reset(self,bb,events):
        self.cond1.reset(bb,events)
        self.cond2.reset(bb,events)

    def __str__(self) -> str:
        return "(" + str(self.cond1) + " or " + str(self.cond2) + ")"

class AndCondition(Condition):
    """
    Combines condition1 and condition2 using AND.

    Typically constructed by applying the operator `&` to two objects.

    - Does not consume from the if the and returns None.
    - Consumes both if the AND returns anything.
    - The result of cond2 will be returned.
    """
    def __init__(self, cond1, cond2):
        self.cond1 = cond1
        self.cond2 = cond2

    def poll(self,bb,events) -> str:
        r1 = self.cond1.peek(bb,events)
        if r1:
            r2 = self.cond2.poll(bb,events) 
            if r2:
                r1 = self.cond1.poll(bb,events)
            return r1 and r2
        else:
            None

    def peek(self,bb,events) -> str:
        return self.cond1.peek(bb,events) and self.cond2.peek(bb,events)

    def reset(self,bb,events):
        self.cond1.reset(bb,events)
        self.cond2.reset(bb,events)

    def __str__(self) -> str:
        return "(" + str(self.cond1) + " and " + str(self.cond2) + ")"

class Not(Condition):
    def __init__(self, event,cond):
        """
        - returns `event` if underlying `cond` returns None.
        - returns None if underlying `cond` returns a string
        """
        self.cond = cond
        self.event = event

    def poll(self,bb,events) -> str:
        if self.cond.poll(bb,events) is None:
            return self.event
        else:
            return None
        
    def peek(self,bb,events) -> str:
        if self.cond.peek(bb,events) is None:
            return self.event
        else:
            return None  
        
    def reset(self,bb,events):
        self.event.reset(bb,events)

    def __str__(self) -> str:
        return "Not('{self.event}',{str(self.cond)} )";


class Callback_Condition(Condition):
    """
    checks wether a callback function returns True.
    """

    def __init__(self, event, cb:Callable[ [Dict ],bool] ):
        """
        Parameters:
            event:
                event to return when callback returns true
            cb:
                callback fun(Blackboard) -> bool, will be called to check for the condition.

        callback_Condition does **never consumes** the event of cb(bb)==True
        """
        self.event          = event
        self.cb             = cb
   
    def poll(self,bb,events):
        if (self.event in events) and self.cb(bb):
            return self.event
        return None
                
    def peek(self,bb,events):
        if (self.event in events) and self.cb(bb):
            return self.event
        return None
            
    def reset(self,bb,events):
        pass

    def __str__(self) -> str:
        return f"callback_Condition('{self.event}', callback)"


class Timeout_Condition(Condition):
    """ 
    Triggers a condition after a gtiven time, can repeat itself.
    """
    def __init__(self, event:str,sec:float, repeat:int=1, consume=True):
        """
        Parameters:
            event:
                name of the event to send out
            sec:
                send the event after `sec` seconds.
            repeat:
                the number of times to repeat this.  Negative to never stop.
            consume:
                whether to consume the event. Can be useful not to consume if you
                just want to check whether you are before or after a given time.
        """
        self.sec         = sec
        self.event       = event
        self.repeat      = repeat
        self.times_left  = repeat
        self.trigger     = 0 
        self.consume     = consume
    
    def poll(self,bb,events):        
        if (self.event in events) and (self.times_left!=0):            
            now = time.monotonic()
            if ( now >= self.trigger):
                if self.consume:
                    self.times_left = self.times_left - 1
                    self.trigger = now + self.sec
                return self.event
        return None
                            
    def peek(self,bb,events):
        if (self.event in events) and (self.times_left!=0):            
            now = time.monotonic()
            if ( now >= self.trigger):                
                return self.event
        return None

    
    def reset(self,bb,events):
        self.trigger          = time.monotonic() + self.sec
        self.times_left       = self.repeat

    def __str__(self) -> str:
        return f"Timeout_Condition('{self.event}', {self.sec})"


class HTTPEvent_Condition(Condition):
    """ Check whether HTTP events come in """
    def __init__(self,channel_name:str='betfsm', queue_size:int=10, max_age:float=0.5, consume=True):
            """

            Parameters:
                channel_name:
                    Name of the channel to listen to.
                queue_size
                    Size of the queue.  If the HTTPReceiver already exists, its queue_size
                    will be enlarged if required.  The queue size is shared with all
                    HTTPEvent_Conditions that use the same channel.
                max_age:
                    only HTTP event with an age less than max_age will be taken into account.
                    The max_age is determined for each HTTPEvent_Condition separately.
                consume:
                    whether or not to consume the HTTP Event
            """            
            self.channel_name   = channel_name
            self.queue_size     = queue_size            
            self.max_age        = max_age
            self.consume        = consume
            self.recv = HTTPEventReceiver.get_instance(channel_name,queue_size)

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
        return f"HTTP_Condition({self.channel_name}, {self.queue_size}, {self.max_age}, {self.triggered_once} )"


class Ctrl_C_Condition(Condition):
    """ Check whether CTRL_C events come in """
    def __init__(self,event="CTRL_C", repeated=3, consume=True):          
            """
            Parameters:
                event:
                    the name of the event to return if Ctrl-C was pressed
                repeated:
                    if the users presses `repeated` times Ctrl-C, the original Ctrl-C handler will be
                    called and the program will be interrupted.  
                consume:
                    if true, the ctrl-c event will be consumed, i.e. if asked again this object will return None.
            """  
            self.event          = event
            self.repeated       = repeated
            self.consume        = consume
            self.recv           = Ctrl_C_Receiver.get_instance(event, repeated)

    def poll(self,bb,events):
        if self.consume:
            return self.recv.poll_for(events)
        else:
            return self.recv.has_event(events)
        
    def peek(self,bb,events):
        return self.recv.has_event(events)
    
    def reset(self,bb,events):
        pass

    def __str__(self) -> str:
        return f"Ctrl_C_Condition({self.event}, {self.repeated}, {self.consume} )"
   







#####################################################################
#                   Event Checkers
#####################################################################


class EventOutcome(Generator):
    """

    Same as Event, but no subtrees in the event_map, only outcomes.
    Is easier to understand.
    """
    def __init__(self, 
                 name:str, 
                 event_poller: Condition,
                 event_map=Dict[str, str] ):
        assert(isinstance(event_poller,Condition))       
        outcomes          = []  # outcomes of Event()        
        if NO_EVENT not in event_map:
            event_map[NO_EVENT] = TICKING                # default is to wait until event arrives:
        for k,v in event_map.items():
            outcomes.append(v)
        self.event_map    = event_map
        self.events       = [e for e,v in self.event_map.items()]
        self.event_poller = event_poller
        
        super().__init__(name, outcomes) # duplicate outcomes are taken care of by super()

    
    def co_execute(self, blackboard):
        self.event_poller.reset(blackboard, self.events)
        while True:
            ev = self.event_poller.poll(blackboard, self.events )
            if ev is None:
                ev = NO_EVENT
            outcome = self.event_map[ev]
            if outcome!=TICKING:
                break
            yield TICKING
        yield outcome
        


class EventConcurrent(GeneratorWithList):
    """    

    maps events to execution of subtrees.

    Similar to ConcurrentSequence but the children start not activated and all
    not active children are can be activated by an event.

    A subtree matched with NO_EVENT starts also not activated, but is activated just after the
    first polling.  

    NO_EVENT is special and should not occur as incoming event. if NO_EVENT is not specified then
    SubtreeEvent keeps waiting (TICKING) until one of the event subtrees returns something else than SUCCEED.

    The NO_EVENT subtree will not be repeated.  the subtrees associated with events can be repeated but the same
    subtree is never executed concurrently multiple times.  subtrees associated with multiple events are executed 
    concurrently

    See Also:
        [events](events.md) for a discussion on usage patterns.

    ```mermaid
    stateDiagram-v2     
        direction TB       
        classDef successClass  fill:darkgreen,color:white
        classDef tickingClass  fill:yellow,color:black
        classDef otherClass  fill:darkorange,color:white
        classDef abortClass  fill:darkred,color:white

        
        state "TICKING <br> if any TICK transition <br> is received" as TICKING
        state "OTHER <br>returns first other outome" as OTHER
        state "SUCCEED <br>if both transitions <br> are received" as SUCCEED

        state fork_state <<fork>> 
        [*] --> fork_state   
        fork_state --> NO_EVENT_subtree  : NO_EVENT
        fork_state --> EVENT_1_subtree : EVENT_1
        EVENT_1_subtree --> EVENT_1_succeeded :  SUCCEED
        EVENT_1_succeeded --> EVENT_1_subtree : EVENT_1
        state join_state <<fork>>
        NO_EVENT_subtree --> join_state : SUCCEED    
        NO_EVENT_subtree --> OTHER : OTHER outcome    
        EVENT_1_succeeded --> join_state 
        EVENT_1_subtree --> OTHER : OTHER outcome
        join_state --> SUCCEED
        NO_EVENT_subtree --> TICKING : TICKING
        EVENT_1_subtree --> TICKING : TICKING
        


        

        class SUCCEED successClass
        class OTHER otherClass
        class TICKING tickingClass
        class TIMEOUT abortClass
    ```


    """
    def __init__(self, 
                 name:str, 
                 event_poller:Condition, 
                 event_map=Dict[str, TickingState] ):  
        assert(isinstance(event_poller,Condition))       
        children                     = []  # list of TickingStates                
        self.event_to_state_ndx      = {} # str -> int        event to index in self.states
        index                        = 0        
        for k,v in event_map.items():
            children.append(v)
            if k!=NO_EVENT:
                self.event_to_state_ndx[k] = index
            index = index + 1
        self.event_poller = event_poller
        self.nominal_state = event_map.get(NO_EVENT,None)
        if self.nominal_state is None:  ########################################
            self.nominal_state = AlwaysOutcome(TICKING)
        self.add_state_safeguard = False
        super().__init__(name, [], children)
        self.add_state_safeguard = True


    def add_state(self, state:TickingState):
        """Not defined for an EventSequential"""
        if self.add_state_safeguard:
            raise AttributeError("do not use EventSequential.add_state(...), only add children via the event_map ")
        super().add_state(state)

    def co_execute(self,blackboard):
        for s in self.states:
            s["active"] = False
        state_events = [  k for k,v in self.event_to_state_ndx.items() if self.states[v]["active"]==False ]
        self.event_poller.reset(blackboard, state_events)            
        countActive = 0
        nominal_succeeded = False
        while True:                    
            # poll events related to non-active states.
            while True:
                state_events = [  k for k,v in self.event_to_state_ndx.items() if self.states[v]["active"]==False ]
                ev = self.event_poller.poll(blackboard, state_events )
                if ev is None:
                    break
                ndx = self.event_to_state_ndx[ev]
                self.states[ndx]["active"] = True                    
                countActive = countActive + 1                                                
            # handle nominal state and flag if it would be ticking:
            ticked = False                            
            if self.nominal_state == None:
                ticked=True
            else:
                outcome = self.nominal_state(blackboard)
                if outcome == TICKING:
                    ticked=True
                elif outcome == SUCCEED:                    
                    nominal_succeeded = True
                else:
                    # this could have interrupted some states:
                    for s in self.states:
                        s["state"].reset()
                    # returning outcome != TICKING interrupts EventConcuurent.
                    yield outcome                
            # for all concurrent states, look for the active ones:
            for s in self.states:
                if s["active"]==True:
                    outcome = s["state"](blackboard)
                    if outcome==TICKING:
                        ticked=True
                    elif outcome==SUCCEED:
                        countActive=countActive-1
                        s["active"]=False  
                    else:
                        # in case any of the underlying states was not completed:
                        for s in self.states:
                            s["state"].reset()
                        yield outcome  # yielded before any tick before or after in the sequence
            if ticked:
                yield TICKING
            if (countActive <= 0) and nominal_succeeded:
                break        
        for s in self.states:   # all of the underlying states have necessarily completed, not needed, just to be safe...
            s["state"].reset()        
        yield SUCCEED

class EventSequential(GeneratorWithList):
    """
    EventSequential checks events in a sequential way. It can execute a nominal subtree or wait by
    ticking forever. If there are incoming events, these are queued in a FIFO way and the corresponding
    subtree is executed one by one.  If any of the subtrees returns with an outcome different from TICKING or SUCCEED
    EventSequential finishes with that outcome.

    The nominal state can be interrupted by the events, will pause while the subtrees corresponding to the events
    are executing, and, if all return SUCCEED, will resume after all subtrees are processed.

    Typical use: 
    - interrupt nominal state by event with cleanup 
    - waiting when sequentially executing the subtrees associated with incomming events.
    """

    # Approach:
    #  NO_LIMITS can be interrupted.
    #  subtrees related to other events are executed sequentially
    #  if NO_LIMITS not specified, similar to: NO_LIMITS = AlwaysOutcome(TICKING)
    #  basically nominal_state and event_state, where event_state can still interrupt nominal state.
    #  events are put on a FIFO stack
    # (1) if nominal_state executing: check stack, if not empty take of to event_state and execute this in place.
    # (2) if event_state is done: event_state=None, and go to (1)

    def __init__(self, 
                 name:str, 
                 event_poller:Callable[[Dict, List[str]],str|None], 
                 event_map=Dict[str, TickingState]
                 ):
        """

        

        Parameters:
            name:
                name of this node
            event_poller:
                polling function from which we get the events.
            event_map:
                A map that maps events to BeTFSM subtrees.   The subtree
                corresponding to the NO_EVENT entry is the nominal subtree.
                If there is no NO_EVENT entry specified, EventSequential will
                tick forever while listening to incomming events.
        """
        assert(isinstance(event_poller,Condition))       
        children                     = []  # list of TickingStates                
        self.event_to_state_ndx      = {}  # str -> int        event to index in self.states
        index                        = 0
        self.nominal_state           = None
        for k,v in event_map.items():            
            if isinstance(v,str):
                v = AlwaysOutcome(name=None,outcome=v)
            children.append(v)
            if k!= NO_EVENT:
                self.event_to_state_ndx[k] = index
            else:
                self.nominal_state = v                
            index = index + 1
        self.event_poller  = event_poller
        
        self.add_state_safeguard = False
        super().__init__(name, [], children)
        self.add_state_safeguard = True


    def add_state(self, state:TickingState):
        """Not defined for an EventSequential"""
        if self.add_state_safeguard:
            raise AttributeError("do not use EventSequential.add_state(...), only add children via the event_map ")
        super().add_state(state)


    def co_execute(self,blackboard):
        # define a FIFO stack of all states that will interrupt the nominal state
        # and will be executed sequentially        
        stack = deque()
        state_events = [  k for k,_ in self.event_to_state_ndx.items() ]
        self.event_poller.reset(blackboard, state_events)
        while True:
            # consume all allowable events  (less efficient with many matching events, no choice due to API)
            while True:
                ev = self.event_poller.poll(blackboard, state_events )
                if ev is None:
                    break
                ndx = self.event_to_state_ndx[ev]
                stack.appendleft(self.states[ndx])
            # execute stack or nominal_state:
            if stack:
                outcome = stack[-1]["state"](blackboard)
                if outcome==SUCCEED:
                    stack.pop()
                elif outcome==TICKING:
                    yield TICKING
                else:
                    # reset states that will be interrupted:
                    for s in self.states:
                        s["state"].reset()
                    # interrupt by yielding != TICKING
                    yield outcome
            else:
                if self.nominal_state is None:
                    yield TICKING
                else:
                    yield self.nominal_state(blackboard)

