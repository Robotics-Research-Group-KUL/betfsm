# tickingstatemachine.py
#
# Copyright (C) Erwin Aertbeliën,  2024
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


# proposed additions/adaptations of etasl_yasmin_utils by Erwin.
#
# Draft


# from simple_node import Node
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin.state import State
from typing import Dict, List, Union, Callable

from yasmin_etasl.yasmin_ticking import Visitor
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT, CANCEL
#from yasmin_ros import ServiceState

#from yasmin_viewer import YasminViewerPub



#from lifecycle_msgs.msg import Transition

# from lifecycle_msgs.srv import ChangeState_Response

#from std_msgs.msg import String


from functools import partial

from typing import List, Callable, Union, Type

#from event_state import EventState



#from etasl_yasmin_utils import *

from enum import Enum
from threading import Lock

import traceback

from rclpy import qos
from rclpy.duration import Duration

from abc import ABC, abstractmethod
from yasmin.blackboard import Blackboard


from .yasmin_ticking import *




class Queue(ABC):
    @abstractmethod
    def push_outcome(self, priority:int, count:int, outcome:str,msg)->None:
        """
        pushes an outcome to the queue, using a standard set of parameters (not always used for every 
        type of Queue)

        Parameters:
            - priority: 
                Priority level, lower is higher priority, 0 is the priority level of the current outcome of a state.
            - int:
                sequence number of messages
            - outcome:
                outcome 
            - msg:
                the message structure (for extra processing for payload)
        """                
        raise Exception("abstract method not implemented")
    
    # @abstractmethod
    # def pop_outcome(self, outcomes:List[str])  -> tuple[int, int, str,Type]:
    #     raise Exception("abstract method not implemented")

    @abstractmethod
    def adapt_outcome(self, outcome:str,outcomes:List[str]) -> tuple[str,Type]:
        """
        Adapts the outcome of a state according to the policy of the queue
        Parameters:
            outcome:
                current outcome of the current state
            outcomes:
                list of allowable outcomes for the current state.
        Returns:
            tuple: with following members
             - outcome
             - message structure (can be None if outcome is not originating from a message)
        """        
        raise Exception("abstract method not implemented")
    
    @abstractmethod
    def clear(self):
        """
        Empties the queue.
        """
        raise Exception("abstract method not implemented") 

    @abstractmethod
    def size(self):
        """
        Returns the length of the queue.
        """
        raise Exception("abstract method not implemented") 


from collections import deque 
class QueueFIFO(Queue):
    """
    A pure first in first out queue that a `Listener` can use to store received messages.
    If the maximum size is exceeded, the oldest messages will be thrown away.
    """
    def __init__(self, maxsize=100):
        self.maxsize=maxsize
        self.q = deque()

    def push_outcome(self, priority: int, count: int, outcome: str, msg) -> None:

        if len(self.q)==self.maxsize:
            self.q.popleft()
        self.q.append((priority, count, outcome,msg))
        


    def adapt_outcome(self, outcome:str,outcomes:List[str]) -> tuple[str,Type]:
        # if no candidate pass through outcome
        if len(self.q)==0:
            return (outcome, None)
        candidate = self.q[0]
        # if candidate has higher priority (lower number):
        if (outcome==TICKING) or (candidate[0]<0):
            del self.q[0]  # consume message
            return (candidate[2],candidate[3])        
        else:
            return (outcome,None)

    def clear(self):
        self.q.clear()

    def size(self):
        """
        size of the queue
        """
        return len(self.q)



class QeueuFIFOfilter(Queue):
    """
    A pure first in first out queue that a `Listener` can use to store received messages.
    It filters the reults using an outcomes list, but does not throw away elements that did not match, leaving 
    them for later use by a state that does recognize the outcome of these elements.

    If the maximum size is exceeded, the oldest messages will be thrown away.
    """    
    def __init__(self, maxsize=100):
        """
        A message queue with first in first out policy
        It filters the result using an outcomes list, but does not throw away the element if it does not
        matches.

        Parameters:
            maxsize:
                maximum size, if longer the queue starts to forget the oldest elements.        
        """
        self.maxsize=maxsize
        self.q = deque(self.maxsize)

    def push_outcome(self, priority: int, count: int, outcome: str, msg) -> None:
        if len(self.q)==self.maxsize:
            self.q.popleft()        
        self.q.append((priority, count,outcome,msg))

    def adapt_outcome(self, outcome:str,outcomes:List[str]) -> tuple[str,Type]:
        if outcome==TICKING:
            # lookup candidate
            count=0
            while count < len(self.q):
                r = self.q[count]
                count=count+1
                if r[2] in outcomes:
                    # candidate found
                    del self.q[count]
                    return (r[2],r[3])
            return (TICKING,None)
        else:
            # lookup candidate (taking into account priority)
            count=0
            while count < len(self.q):
                r = self.q[count]
                count=count+1
                if (r[2] in outcomes) and (r[0] < 0):
                    # candidate found
                    del self.q[count] # consume message
                    return (r[2],r[3])
            return (outcome,None)
    
    def clear(self):
        self.q.clear()
    
    def size(self):
        return len(self.q)






class Listener(ABC):
    """
    The abstract base class Listener encapsulates an object than can listens for messages and put it on a queue.
    The state machine can use this to inject additional transitions into the state machine.
    """
    def __init__(self,outcomes: List[str],queue:Queue=QueueFIFO()):
        """
        maintains a heap (priority, entrycount, outcome7)

        prioriities: 0 is the priority of the outcome of the current state, 
        negative is more priority than current outcome, 
        positive is lower priority than the current outcome (but still used when outcome==TICKING)

        Note:
         - Only the adapt_outcome(), start() and stop() methods are used by the statemachine
         - set_payload is overwritten by the subclass and and push_outcome can be used by subclasses, 
           it it is called by adapt_outcome()

        """        
        self.outcomes = outcomes
        self.queue = queue
        pass

    def adapt_outcome(self, blackboard: Blackboard, outcome:str,outcomes:List[str]) -> str:
        """
        Possibly adapts the outcome of a state depending on priority rules and queuing pollicy of the underlying queue.

        Parameters:
            blackboard:
                blackboard
            outcome:
                current outcome, can be ignored if there is a higher priority message.
            outcomes:
                list of eligible outcomes.
        """
        outcome, msg = self.queue.adapt_outcome(outcome,outcomes)
        if msg is not None:
            self.set_payload(blackboard,msg)
        return outcome
        
    @abstractmethod
    def set_payload(self, blackboard: Blackboard, msg):
        """
        uses the currently used message to set the payload in the blackboard
        (method only to be used by subclasses)
        """
        raise NotImplementedError("set_payload abstract method is not implemented by subclass")

    @abstractmethod
    def start(self):
        """
        starts listening for messages from a source
        """
        raise NotImplementedError("start() abstract method is not implemented by subclass")

    @abstractmethod
    def stop(self):
        """
        stops listening for messages from a source
        """
        raise NotImplementedError("stop() method is not implemented by subclass")
    

class StateMachineElement:
    """
    Just to have a type that a visitor could recognize
    """
    def __init__(self,name,state,transitions):
        self.name = name
        self.state = state
        self.transitions = transitions

    def accept(self, visitor: Visitor):
        if visitor.pre(self):
            self.state.accept(visitor)
        visitor.post(self)

class TickingStateMachine(TickingState):
    """
    A version of StateMachine that calls a callback function before entering a state and/or at each transition.
    extended version of the cbStateMachine from yasmin_action

    Constructor(outcomes,transitioncb,statecb) :
        - outcomes: the allowed outcomes of the state machine, cause the state machine to exit and return one of these outcomes   
        - transitioncb: callback function called at each transition. 
          Signature transitioncb(source_state:str, transtion:str, target_state:str)
        - statecb: callback function called before entering each state. 
          Signature statecb(name)

    *In the case of a multithreaded application, it is assumed that the callback functions are reentrant or protected with a lock*

    This class is useful but not necessary when using it with an ROS2 Action Server. Examples of usage:
     - log transitions to ros2's logger 
     - published action feedback on transitions
    
     
    This statemachine is capable of working together with TickingState:
      - will exit when TICKING outcome is given by one of the substates, but then if it is called again,
        it will have remembered the state that had the TICKING outcome and start from that state.
      - if returning with any other outcome, will start next time from the start state.
      - should be drop in replacement of Yasmin StateMachine, (as long as nobody uses TICKING as outcome.)
    """    
    def __init__(self, name:str, outcomes: List[str], listener:Listener=None, transitioncb=default_transitioncb, statecb=default_statecb) -> None:
        outcomes.append(TICKING)
        super().__init__(name,outcomes)

        self._states = {}
        self._start_state = None
        self.__current_state = None
        self.__current_state_lock = Lock()
        self.statecb = statecb
        self.transitioncb = transitioncb
        self.listener = listener        
        
    def add_state(
        self,
        name: str,
        state: State,
        transitions: Dict[str, str] = None
    ) -> None:
        if not transitions:
            transitions = {}
        if not isinstance( transitions , Dict):
            raise ValueError("transitions should be a dictionary")            
        self._states[name] = {
            "state": state,
            "transitions": transitions
        }

        if not self._start_state:
            self._start_state = name
            self.__current_state = name
        
        
    def set_start_state(self, name: str) -> None:
        self._start_state = name
        self.__current_state = name

    def get_start_state(self) -> str:
        return self._start_state

    def cancel_state(self) -> None:
        super().cancel_state()
        with self.__current_state_lock:
            if self.__current_state:
                self._states[self.__current_state]["state"].cancel_state()

    def reset(self):
        with self.__current_state_lock:
            state = self.__current_state
            if isinstance(state,TickingState):
                state["state"].reset()
            self.__current_state = self._start_state
        super().reset()

    def accept(self, visitor: Visitor):
        if visitor.pre(self):
            for k,v in self._states.items():
                StateMachineElement(k,v["state"],v["transitions"]).accept(visitor)
        visitor.post(self)

    def entry(self, blackboard: Blackboard) -> str:
        self.listener.start()
        return CONTINUE

    def doo(self, blackboard: Blackboard) -> str:
        #with self.__current_state_lock:
        #    self.__current_state = self._start_state
        while True:
            with self.__current_state_lock:
                state = self._states[self.__current_state]
                name = self.__current_state
            self.statecb(self,blackboard,name)
            outcome = state["state"](blackboard)
            # check outcome belongs to state (double check on sanity of state, although state base already checks this)
            if outcome not in state["state"].get_outcomes():
                with self.__current_state_lock:
                    self.__current_state = self._start_state
                raise Exception(
                    f"Outcome ({outcome}) is not register in state {self.__current_state}")
            
            # construct list of allowable incomes for possible filtering by listener:
            #   - outcomes mentioned in transtion,
            #   - outcomes of the underlying state
            #   - outcomes of the state machine (i.e. going out)
            #   - outcome is directly the name of a state
            if self.listener is not None:
                allowable_outcomes = [ t for o,t in state["transitions"].items() ] + \
                                    state["state"].get_outcomes() + \
                                    self.get_outcomes() + \
                                    [ sname for sname,s in self._states.items() ]
                old_outcome=outcome
                outcome = self.listener.adapt_outcome(blackboard,outcome, allowable_outcomes  )
                #if outcome !=old_outcome:
                #    print(f"queue size {self.listener.queue.size()}")
                #    print(f"calling adapt_outcome with {old_outcome}\n\t\t\t{allowable_outcomes}\nresulting in {outcome}")

            outcome = self.transitioncb(self,blackboard,self.__current_state, outcome)
            # translate outcome using transitions
            if outcome in state["transitions"]:              
                outcome = state["transitions"][outcome]
            if outcome == TICKING:                # outcome is TICKING and exits state machine but keeps current state                                                 
                return outcome
            elif outcome in self.get_outcomes():      # outcome is an outcome of the sm, reset current state
                with self.__current_state_lock:
                    self.__current_state = self._start_state
                return outcome 
            elif outcome in self._states:           # outcome is a state
                with self.__current_state_lock:
                    self.__current_state = outcome
            else:                                   # outcome is not in the sm
                with self.__current_state_lock:
                    self.__current_state = self._start_state                                                                            
                raise Exception(f"""State {name} has outcome ({outcome}) without transition
                                    transitions {state['transitions']}
                                    outcomes state machine {self.get_outcomes()} """)
    
    def exit(self) -> str:
        self.listener.stop()
        with self.__current_state_lock:
            state = self.__current_state
            if isinstance(state,TickingState):
                state["state"].reset()
            self.__current_state = self._start_state
        return super().exit()
    
        
    def get_states(self) -> Dict[str, Union[State, Dict[str, str]]]:
        return self._states

    def get_current_state(self) -> str:
        with self.__current_state_lock:
            if self.__current_state:
                return self.__current_state

        return ""

    def __str__(self) -> str:
        return f"StateMachine: {self._states}"
    


