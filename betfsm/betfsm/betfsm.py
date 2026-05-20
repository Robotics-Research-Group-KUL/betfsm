# betfsm.py
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



from typing import Dict, List, Union, Callable,Type, TypeAlias,Iterable,Optional
from enum import Enum
import copy
from threading import Lock
from abc import ABC, abstractmethod
import traceback
import uuid
import time 
import json
from .logger import get_logger,add_logger_category,get_logger_categories

import signal,math
from collections import deque
from dataclasses import dataclass

add_logger_category("state")
add_logger_category("service")




# Track which classes have already shown their deprecation notice
from colorama import Fore, Back, Style
import inspect



__shown_deprecation_for_class = {}

def deprecated_msg(msg):
    """
    Prints a deprecation warning once per class, including:
    - module name
    - class name
    """
    # Inspect the call stack
    frame = inspect.currentframe()
    caller = frame.f_back

    module = caller.f_globals.get("__name__", "<unknown module>")
    func_name = caller.f_code.co_name

    # Try to detect the class (works when called inside a method)
    cls = caller.f_locals.get("self", None).__class__ if "self" in caller.f_locals else None
    cls_name = cls.__name__ if cls else "<no class>"

    # Only print once per class
    if cls and cls not in __shown_deprecation_for_class:
        print(
            f"{Fore.RED}{Style.BRIGHT}DEPRECATION WARNING:{Style.RESET_ALL} "
            f"{module}.{cls_name} is deprecated and will be removed in the future.\n{Fore.RED}{Style.BRIGHT}{msg}{Style.RESET_ALL}"
        )
        __shown_deprecation_for_class[cls] = True



Blackboard: TypeAlias = Dict[str, Dict|any]


def cleanup_outcomes(outcomes:List[str])->List[str]:
    """
    cleans up a list of outcomes by elliminating duplicates.

    Parameters:
        outcomes:
            list of outcomes with possibly duplicate elements.

    Returns:
        list of outcomes with duplicate elements elliminated.
    """
    return [ e for e in {e for e in outcomes}]


#
# outcomes with some special meaning for some TickingStates
# just to allow a more systematic definition of states
#

SUCCEED  = "SUCCEED"    
""" outcome to indicate that everything is fine, continue as normal"""

ABORT    = "ABORT"      
""" outcome to indicate an involuntary stop, e.g. due to exception raised, communication failure,..."""

CANCEL   = "CANCEL"     
""" outcome to indicate a voluntary stop, deliberatly provoked, e.g. reacting to cancel request of an action"""

TIMEOUT  = "TIMEOUT"      
""" outcome to indicate that some operation has timed out. """

TICKING  = "TICKING"      
""" outcome to indicate that you expect to called back the next tick (internally used when yielding)"""

CONTINUE = "CONTINUE"     
""" outcome only to be used in the entry() method of TickingState, to signal tht you want 
    to directly continue with Doo. Do not use it anywhere else
"""

NO_EVENT = "NO_EVENT"
"""outcome used in the eventmap of the Event... classes to indicate what to do when there is no event in the current tick."""

TickingState_Status = Enum("TickingState_Status",["ENTRY","DOO","EXIT"])



def get_path_value(blackboard, path, default=None):
    """
    Gets a value in the blackboard at the given path,.

    Parameters:
        blackboard:
            Blackboard Dict
        path:
            path to get the value of
        default:
            value to use if not found, by default = None
        delimiter:
            for the path, default='/'
    Returns:
        value:
            the value at the given location

    Note:
        See [Utilities](utils.md) for a more extensive explanation of the mini-language to specify and manipulate the path.
    """
    delimiter="/"
    keys = [k for k in path.split(delimiter) if k]
    current = blackboard

    for key in keys:

        # LIST LENGTH
        if key == "~len":
            if isinstance(current, list):
                return len(current)
            return default

        # LIST INDEX
        if isinstance(current, list) and key.isdigit():
            idx = int(key)
            if 0 <= idx < len(current):
                current = current[idx]
            else:
                return default
            continue

        # NORMAL DICT ACCESS
        if isinstance(current, dict) and key in current:
            current = current[key]
            continue

        # UNKNOWN PATH
        return default

    return current



def set_path_value(blackboard, path, value):
    """
    Sets a value in the blackboard at the given pat.

    Parameters:
        blackboard:
            Blackboard Dict
        path:
            path to get the value of
        value:
            value to fill in.

    Warning:
        if value is a Dict, only a shallow copy is made! So, if you later
        change an element of value, it will also change the Blackboard.
        you can use copy.deepcopy(subtree) to make a deep copy to avoid this.
    
    Note:
        See [Utilities](utils.md) for a more extensive explanation of the mini-language to specify and manipulate the path.
    """    
    delimiter='/'
    keys = [k for k in path.split(delimiter) if k]
    current = blackboard

    # Traverse until the parent of the last key
    for i, key in enumerate(keys[:-1]):
        next_key = keys[i + 1]
        # If next is list operation, ensure list exists
        if next_key.isdigit() or next_key.startswith("~"):
            if key not in current or not isinstance(current[key], list):
                current[key] = []
            current = current[key]
            continue
        # Normal dict creation
        if key not in current or not isinstance(current[key], dict):
            current[key] = {}
        current = current[key]
    last = keys[-1]
    # LIST OPERATIONS
    # APPEND
    if last == "~append":
        if not isinstance(current, list):
            raise TypeError("Cannot append to non-list")
        current.append(value)
        return
    # POP
    if last == "~pop":
        if not isinstance(current, list):
            raise TypeError("Cannot pop from non-list")
        if current:
            current.pop()
        return
    # DELETE INDEX
    if last.startswith("~del:"):
        idx = int(last.split(":", 1)[1])
        if not isinstance(current, list):
            raise TypeError("Cannot delete from non-list")
        if 0 <= idx < len(current):
            del current[idx]
        return
    # INSERT INDEX
    if last.startswith("~insert:"):
        idx = int(last.split(":", 1)[1])
        if not isinstance(current, list):
            raise TypeError("Cannot insert into non-list")
        current.insert(idx, value)
        return
    # REPLACE INDEX
    if last.isdigit():
        idx = int(last)
        if not isinstance(current, list):
            raise TypeError("Cannot index non-list")
        # auto-extend list
        while len(current) <= idx:
            current.append(None)
        current[idx] = value
        return

    
    # NORMAL DICT SET
    
    current[last] = value




def get_path_location(blackboard, path, create_if_needed = True, delimiter='/'):
    """
    gets a location in the blackboard at the given path

    Parameters:
        blackboard:
            Blackboard Dict
        path:
            path to get the value of
        create_if_needed:
            path will be created if needed, overwriting
            any values if needed.
        delimiter:
            for the path, default='/'
    """
    keys = [k for k in path.split(delimiter) if k]   
    current = blackboard
    for i, key in enumerate(keys[:-1]):
        if key not in current or not isinstance(current[key], dict):
            current[key] = {}
        current = current[key]
    if keys:
        if key[-1] not in current  or not isinstance(current[key], dict):
            current[keys[-1]] = {}
        return current[keys[-1]]
    else:
        return blackboard











# """
#     ```graphviz

#         digraph monitoringstate{
#             //node [shape=point] start;      
#             //node [shape=point] end;
#             node [shape=point] start
#             node [shape=box, style=rounded];
        

#             node [label= "  entry()\n if TICK return TICK\n if exception return ABORT"] Entry;

#             node [label= "  doo()\n if TICK return TICK\n if exception return ABORT"] Doo;
#             node [label= "  exit()\n return outcome \n if exception return ABORT"] Exit;
#             start->Entry

#             Entry -> Doo [label="CONTINUE\nor TICK"]

#             Entry -> Exit [label="≠TICK and\n ≠CONTINUE\nor ABORT"]
#             Doo -> Exit [label="≠TICK\nor ABORT"]
#             Doo -> Doo [label="TICK"]
#             Exit -> Entry 
        
#         }
#     ```
# """

class Visitor(ABC):
    """
    Visitor pattern.

    Called by accept() methods of TickingState and its subclasses. The Visitor is a base class that you
    can inherit from to have your object called throughout the hierarchy of states.  This mechanism
    is completely generic.

    See also:
        - *Erich Gamma, Richard Helm, Ralph Johnson, John Vlissides (1994). Design Patterns: Elements of Reusable Object-Oriented Software. Addison Wesley.* (Gang of Four book)
        - [wikipedia](https://en.wikipedia.org/wiki/Visitor_pattern)


    """
    @abstractmethod
    def pre(self,state) -> bool:
        """
        Processing done by visitor before visiting the children of the state.

        returns true if children needs to be explored
        """
        raise NotImplementedError("Visitor.pre() method not implemented")
    
    def post(self,state):
        """
        Processing done by the visitor after visiting the children.

        returns nothing.
        """
        raise NotImplementedError("Visitor.pre() method not implemented")






class TickingState:
    """
    Implements a 'ticking' state, i.e. a state that takes a longer time, but cooperatively yields
    the initiative back to the caller (cooperative concurrency):

    ```mermaid
        stateDiagram-v2
            direction LR
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white
            classDef centerClass  text-align:center

        entry --> doo : returns <br>CONTINUE or TICK
        doo --> exit : returns not TICK
        doo --> doo: returns<br>TICK
        entry -->exit: returns <br>not TICK or CONTINUE
        exit  --> entry : exit() finished <br> or reset() called
        doo --> exit : reset() called

        entry: calls entry()<br>if TICK return TICK<br>if exception return ABORT
        doo:   calls doo()<br> if TICK return TICK <br>if exception return ABORT
        exit:  calls exit()<br> return outcome<br>if exception return ABORT

        class entry centerClass
    ```

    note:
        You can choose whether to tick between entry() and doo() by letting entry() return CONTINUE
    
    note:
        exit() is always called when execute is called for the last time, even when there are exceptions.

    note: 
        expects the following methods to be overridden by subclases:

        - entry()
        - doo()
        - exit()
        - reset() (but call super().reset() )

    warning:
        use is similar to `State`, but if it returns TICKING, it is expected to be called again,
        if not, the user needs to call reset() before using the state again.

    """

     # registry of uid -> node map of all existing nodes that have been
     # executed once
    _global_registry = {}


    @classmethod
    def get_global_registry(cls):
        """
            A log where nodes are only added, a single publisher can periodically 
            erase all nodes from this log.  This allows to capture all notes that 
            where active during a period. This is only done when it is turned
            on by assigning an empty dict to global_publish_log
        """        
        return cls._global_registry

    # all the nodes that are painted active
    # something external clears this, BeTFSM trees only add to this, also uid -> node map
    global_publish_log = None 


    _name_sequences={}

    def expand_name(self,name):
        if name is None:
            shortname = self.__class__.__name__
            fqn       = f"{self.__class__.__module__}.{self.__class__.__name__}"
            if fqn not in self._name_sequences:
                seqnr = 1
            else: 
                seqnr = self._name_sequences[fqn]+1
            name = f"{shortname}_{seqnr}"
            #name = f"{shortname}"
            self._name_sequences[fqn] = seqnr
        return name

    def __init__(self,name:str, outcomes: List[str]):
        """
        parameters:
            name:
                name of the TickingState.  This is meant to be an instance-name, not a class-name.
                if equal to None, one will be generated from class name and sequence number.
            outcomes:
                all possible outcomes of the state, TICKING and ABORT will be added.
        """
        self.outcomes = outcomes
        self.outcomes.append(TICKING)
        self.outcomes.append(ABORT)
        for outc in self.outcomes:
            if outc==CONTINUE:
                raise ValueError(f"{name} state:  outcome {CONTINUE} is reserved for internal use!")
        self.name = self.expand_name(name)
        self.parent = None
        self.uid = str(uuid.uuid4())
        self.typename = self.__class__.__name__
        self.fqn      = f"{self.__class__.__module__}.{self.__class__.__name__}"
        self.status = TickingState_Status.ENTRY
        self.outcome = "" # will contain the last used outcome
        TickingState._global_registry[self.uid] = self

    def __call__(self, blackboard: Blackboard) -> str:
        if blackboard is None:
            raise Exception("blackboard argument should not be None")
        outcome = self.execute(blackboard)
        if outcome is None:
            raise Exception(f"execute() method of {self.name} with type  {type(self).__name__} should return a string reprsenting the outcome")
        if outcome not in self.outcomes:
            raise Exception(
                f"Outcome '{outcome}' of  {self.name} with type {type(self).__name__} does not belong to the outcomes of the state {self.outcomes}")
        return outcome


    def reset(self)->None:
        """
        External reset of the TickingState to its initial condition.
        Subclasses should also reset all the children of themselves
        exit() is called when appropriate
        """
        if self.status == TickingState_Status.DOO:
            self.exit()
            get_logger("state").info(f"Exit {self.name} with no outcome")
        self.status = TickingState_Status.ENTRY

    def execute(self, blackboard: Blackboard) -> str:        
        #self.log.info("TickintState.execute")
        if self.status == TickingState_Status.ENTRY: 
            get_logger("state").info(f"Entering {self.name}")
            try:
                self.outcome = self.entry(blackboard)             
                if TickingState.global_publish_log is not None:
                    TickingState.global_publish_log[self.uid] = self
            except Exception as e:
                get_logger().error("exception occurred : "+ traceback.format_exc())
                self.outcome = ABORT
                self.status = TickingState_Status.EXIT
            if self.outcome == TICKING:
                self.status = TickingState_Status.DOO                
                return self.outcome
            if self.outcome == CONTINUE:
                self.status = TickingState_Status.DOO
            else:
                self.status == TickingState_Status.EXIT

        if self.status == TickingState_Status.DOO:
            try:
                self.outcome = self.doo(blackboard)
                if TickingState.global_publish_log is not None:
                    TickingState.global_publish_log[self.uid] = self
            except Exception as e:
                get_logger().error(f"{self.name} : exception occured : "+ traceback.format_exc())
                self.outcome = ABORT
                self.status = TickingState_Status.EXIT
            if self.outcome == TICKING:
                return TICKING
            self.status = TickingState_Status.EXIT

        if self.status == TickingState_Status.EXIT:
            self.outcome = self.exit()
            # EA May 2026: self.status = TickingState_Status.ENTRY
            self.reset()   # EA May 2026, does not call exit since status==EXIT
            get_logger("state").info(f"Exit {self.name} with {self.outcome}")
            return self.outcome
        get_logger("state").info(f"Exit {self.name} with {self.outcome}")
        return self.outcome # in case of ABORT

    def entry(self, blackboard: Blackboard) -> str:
        """
        called the first time execute() is called.

        Parameters:
            blackboard: 

        Returns:
            A string with value:
                CONTINUE (execute will call directly doo() ); 
                TICKING (execute will return and next time will call doo() ); 
                OTHER string  ( exit() will be immediately called, without a tick)

        Note:
            can raise exception, equivalent to returning ABORT. In that case exit() is called.
            If one likes more detailed behavior, entry needs to catch the exception itself.
        """
        return CONTINUE;

    def doo(self, blackboard: Blackboard) -> str:
        """
        is repeatedly called after the first time execute() is called.

        Parameters:
            blackboard: 

        Returns:
            A string with value: TICKING (execute will return and next time will call doo() );
            OTHER string (execute will call immediately exit(), i.e. without a tick )

        Note:        
            if this throws an exception, outcome=ABORT  and and exit() is called immediately
            If one likes more detailed behavior, doo needs to catch the exception itself.        
        """
        return SUCCEED

    def exit(self) -> str:
        """
        method that is always called when execute is called the last time

        Returns:
            A string the outcome (next time execute will call entry() )    

        Note:
            method has no blackboard parameter, since it could be called from reset()
            which does not and should not know the blackboard parameter.

        Warning:
            can't raise an exception!
        """
        # returns by default the outcome of entry or doo method.
        return self.outcome

    def accept(self, visitor:Visitor):
        """
        calls the visitor with itself and possibly iterates over its children.

        See also:
            Visitor        
        """
        visitor.pre(self)
        # no children
        visitor.post(self)


    def __str__(self) -> str:
        return f"{self.name}<{self.__class__.__name__}>"

    def get_outcomes(self) -> List[str]:
        return self.outcomes



class Generator(TickingState):
    """
    Uses a python generator to define a TickingState. It implements methods `entry`, `doo`, and `exit` of TickingState using
    the given callback function (which is a python generator that can return intermediate results usig `yield`).  This makes
    it easy to specify a TickingState.

    Subclasses need to implement/override the abstract method co_execute(self, blackboard:Blackboard)
    """
    def __init__(self, name:str,outcomes: List[str]) -> None:
        """
        Parameters:
            name:
                name of the node
            outcomes:
                a list of strings indicating the expected outcomes,  TICKING and ABORT will be
                automatically added.
        """
        if isinstance(outcomes,str):
            outcomes=[outcomes]
        super().__init__(name,outcomes)        

    def entry(self, blackboard: Blackboard) -> str:
        self.generator = self.co_execute(blackboard)
        self.outcome = SUCCEED
        return CONTINUE

    def doo(self,blackboard:Blackboard) -> str:
        #while True:
        outcome = next(self.generator)        
        if outcome==TICKING:
            return TICKING
        else:
            self.outcome = outcome
            return outcome
    
    def exit(self) -> str:
        if self.outcome is None:
            raise Exception("outcome should not be None")
        return self.outcome
    
    def reset(self):
        super().reset()

    @abstractmethod
    def co_execute(self,blackboard:Blackboard) -> str:
        raise NotImplementedError("Subclasses deriving from Generator need to implement `co_execute(self,blackboard)`")

        


class GeneratorWithList(Generator):
    """
    A generator with some facilities to maintain an ordered list of children
    """
    def __init__(self, name:str, outcomes:List[str], children: List[TickingState]=[]) -> None:
        """
        parameters:
            name: 
                name 
            outcomes:
                list of possible outcomes (in addition to the outcomes of the underlying states)
            children:
                a list of states. 
                you can use add_state(...) to add children.
        """
        super().__init__(name,outcomes)
        self.states=[]
        if (children is not None) and (children != []):
            for c in children:
                self.add_state(c)

    def add_state(self, state: TickingState):
        """
        adds a state to the sequence
        parameters:
            state:
                state intance
        returns:
            self (to allow method chaining)
        """
        if not isinstance(state,TickingState):
            raise ValueError(f"add_state expects as second argument an instance of a subclass of TickingState but got {type(state)}")
        if state.parent is not None:
            raise ValueError(f"{state.name} already belongs to {state.parent.name}")
        state.parent = self
        self.states.append({"name":state.name,"state":state,"active":False}) 
        self.outcomes = cleanup_outcomes(self.outcomes + state.get_outcomes())
        self._outcomes = self.outcomes  # dirty hack to fix a bug
        return self 
    
    def reset(self):  # general rule, if you own states, you have to reset them
        """
        resets the sequence and ensures tht the underlying states are also reset.
        """
        for s in self.states:
            s["active"]=False
            if isinstance(s["state"],TickingState):
                s["state"].reset()
            else:
                raise ValueError(f"{self.name}.reset() : state {s['name']} is not a TickingState, but is of type {type(s['state'])} ")
        super().reset()  

    
    def accept(self, visitor:Visitor):
        if visitor.pre(self):
            for s in self.states:
                s["state"].accept(visitor)
        visitor.post(self)        

       
    @abstractmethod
    def co_execute(self,blackboard):
        raise NotImplementedError("Subclasses deriving from GeneratorWithList need to implement 'co_execute(blackboard)'")
        

class Sequence(GeneratorWithList):
    """
    Implements a behaviortree-like Sequence node:
      - success is SUCCEED outcome,
      - failure is any other outcome

    There is a method `add_state` to add underlying nodes to the Sequence, these are executed
    in order. 
     
    If any node fails, this outcome is directly returned; if a node returns TICKING,
    the sequence returns TICKING; and if the node returns SUCCEED, the execution of the
    next node in the list is started (without any intermediate tick!).
    
    ```mermaid
        stateDiagram-v2                
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white


            [*] --> Loop
            Loop --> state_1 : current_state==1
            Loop --> state_2 : current_state==2
            Loop --> SUCCEED : last state succeeded
            state_1 --> Loop : SUCCEED 
            state_2 --> Loop : SUCCEED

            state_1 --> TICKING : TICKING
            state_2 --> TICKING : TICKING
            

            state_1 --> OTHER : OTHER outcome

            state_2 --> OTHER : OTHER outcome
            

            class SUCCEED successClass
            class OTHER otherClass
            class TICKING tickingClass
            class TIMEOUT abortClass
    ```
    """
    def __init__(self, name:str, children: List[TickingState]=[]) -> None:
        """
        parameters:
            name: 
                name of the sequence
            children:
                a list of states
                you can use add_state(...) to add children.
        """
        super().__init__(name,[],children)        
        self.count = 0        
    
    def co_execute(self,blackboard):        
        for s in self.states:                
            outcome = s["state"](blackboard)
            while outcome==TICKING:
                yield TICKING
                outcome = s["state"](blackboard)
            if outcome!=SUCCEED:
                yield outcome                
        yield SUCCEED



class ConcurrentSequence(GeneratorWithList):
    """
    Implements a behaviortree-like Sequence node:
      - success is SUCCEED outcome,
      - failure is any other outcome

    There is a method `add_state` to add underlying nodes to the Sequence, these are executed
    in order. 

    In contrast to `Sequence` this node runs concurrently. At each call, ConcurrentSequence runs through
    the list of underlying active states and:

      - If an underlying state returns SUCCEED, it becomes inactive. If all states have become inactive,
       the Sequence returns SUCCEED (a "success")
      - If an underlying active state returns another outcome besides TICKING and SUCCEED, the Sequence returns
    the outcome of the first such state (a "fail" ) and cancels the other running states.
      - If any of the states has returned TICKING, the sequence returns TICKING


    In summary, it is similar to Sequence, but it calls all underlying states together (in order), and not one-by-one.
    ConcurrentSequence waits until all underlying states have finished ( a "join")

    ```mermaid
    stateDiagram-v2     
        direction TB       
        classDef successClass  fill:darkgreen,color:white
        classDef tickingClass  fill:yellow,color:black
        classDef otherClass  fill:darkorange,color:white
        classDef abortClass  fill:darkred,color:white

        state fork_state <<fork>> 
        [*] --> fork_state   
        fork_state --> state_1  
        fork_state --> state_2
        state join_state <<fork>>
        state_1 --> join_state : SUCCEED    
        state_1 --> OTHER : OTHER outcome    
        state_2 --> join_state : SUCCEED
        state_2 --> OTHER : OTHER outcome
        join_state --> SUCCEED
        state_1 --> TICKING : TICKING
        state_2 --> TICKING : TICKING
        



        state "TICKING <br> if any TICK transition <br> is received" as TICKING
        state "OTHER <br>returns first other outome" as OTHER
        state "SUCCEED <br>if both transitions <br> are received" as SUCCEED


        class SUCCEED successClass
        class OTHER otherClass
        class TICKING tickingClass
        class TIMEOUT abortClass
    ```
        
    """
    def __init__(self, name:str, children: List[TickingState]=[]) -> None:
        """
        parameters:
            name: 
                name of the sequence
            children:
                a list of states
                you can use add_state(...) to add children.
        """
        super().__init__(name,[],children)
               
    def co_execute(self,blackboard):
        """
        executes the underlying states in sequence, as much as possible concurrently.
        all outcomes except for SUCCEED and TICKING indicate failure.
        SUCCEEDs when all underlying states have returned SUCCEED.
        parameters:
            blackboard:
        """

        for s in self.states:
            s["active"] = True
        countActive = len(self.states)
        while (countActive > 0):
            ticked = False
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
        # all of the underlying states have necessarily completed
        yield SUCCEED
        

class Concurrent(GeneratorWithList):
    """
    Implements Concurrency, the children are executed at each tick concurrently
    until a child returns an outcome different from TICKING. After such an outcome,
    the other children are also reset/stopped.

    ```mermaid
    stateDiagram-v2     
        direction TB       
        classDef successClass  fill:darkgreen,color:white
        classDef tickingClass  fill:yellow,color:black
        classDef otherClass  fill:darkorange,color:white
        classDef abortClass  fill:darkred,color:white

        state fork_state <<fork>> 
        [*] --> fork_state   
        fork_state --> state_1  
        fork_state --> state_2
        
        state_1 --> OTHER : OTHER outcome    
        state_2 --> OTHER : OTHER outcome
        state_1 --> TICKING : TICKING
        state_2 --> TICKING : TICKING
        



        state "returns TICKING <br> when one are more TICK transitions <br> are received" as TICKING
        state "returns OTHER <br>when first outcome that arrives <br>" as OTHER


        class SUCCEED successClass
        class OTHER otherClass
        class TICKING tickingClass
        class TIMEOUT abortClass
    ```
        
    """
    def __init__(self, name:str, children: List[TickingState]=[]) -> None:
        """
        Implements Concurrency, the children are executed at each tick concurrently
        until a child returns an outcome different from TICKING.

        Parameters:
            name: 
                name of the sequence
            children:
                a list of states
                you can use add_state(...) to add children.
        """
        super().__init__(name,[],children)


    def co_execute(self,blackboard:Blackboard):
        """        
        Python generator routine (co-routine). You can use `yield <outcome>` to return intermediate results

        Parameters:
            blackboard:
        """
        while True:
            for s in self.states:
                outcome = s["state"](blackboard)
                if outcome!=TICKING:
                    self.reset()
                    yield outcome
            yield TICKING






class Fallback(GeneratorWithList):
    """    
    Implements a behaviortree-like Fallback node:
      - executes a child and if it is a failure, continues with the next child. Otherwise 
        it returns the outcome of the child that did not fail.  If all children fail
        it returns CANCEL.
      - **definition of failure can be overridden using callback** but by default
        is corresponds to a CANCEL outcome. The failure callback
        can not only depend on the outcome but also on some information in the blackboard

    There is a method `add_state` to add underlying nodes to the Fallback node, these are executed
    in order.  

    If any node succeeds, this outcome is directly returned; if a node returns TICKING,
    the fallback node returns TICKING; and if the node returns CANCEL, the execution of the
    next node in the list is started (without any intermediate tick!).


    ```mermaid
        stateDiagram-v2                
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white


            [*] --> Loop
            Loop --> state_1 : current_state==1
            Loop --> state_2 : current_state==2
            Loop --> CANCEL : last state failed
            state_1 --> Loop : CANCEL
            state_2 --> Loop : CANCEL

            state_1 --> TICKING : TICKING
            state_2 --> TICKING : TICKING
            

            state_1 --> OTHER : OTHER outcome

            state_2 --> OTHER : OTHER outcome
            

            class SUCCEED successClass
            class OTHER otherClass
            class TICKING tickingClass
            class CANCEL abortClass
    ```    
    """
    def __init__(self, name:str, 
                 children: List[TickingState]=[],
                 failure:Callable[[Dict,str], bool] = lambda bb,outc: outc==CANCEL
                ) -> None:
        """
        parameters:
            name: 
                name of the sequence
            children:
                a list of states 
                you can use add_state(...) to add children.
            failure:
                This callback defines what a failure is, can use blackboard or
                outcome. A callback with signature: `def failure(bb:Blackboard,outcome:str)->bool`   

        warning:
            The failure callback should also take into account TICKING outcomes.      
        """
        super().__init__(name,[], children)
        self.failure = failure
        
    def co_execute(self,blackboard):
        for s in self.states:             
            outcome=TICKING
            while outcome==TICKING:
                outcome = s["state"](blackboard)
                if self.failure(blackboard,outcome):
                    get_logger("state").info(f"{self.name} : failure : outcome = {outcome}")
                    outcome=CANCEL  # force end of while loop and go to next child
                else:
                    if outcome!=TICKING:
                        self.reset()    # we are done and will not be comming back.                 
                    yield outcome    # none failure outcome or TICKING
        yield CANCEL

class ConcurrentFallback(GeneratorWithList):
    """
    Implements a behaviortree-like Fallback node:
      - success is any other outcome,
      - failure is CANCEL outcome

    There is a method `add_state` to add underlying nodes to ConcurrentFallback, these are executed
    in order. 

    In contrast to `Fallback` this node runs concurrently. At each call, ConcurrentFallback runs through
    the list of underlying active states and:

      - If an underlying state returns CANCEL, it becomes inactive. If all states have become inactive,
       the ConcurrentFallback returns CANCEL 
      - If an underlying active state returns another outcome besides TICKING and CANCEL, the Sequence returns
    the outcome of the first such state (a "success" ) and cancels the other running states.
      - If any of the states has returned TICKING, the sequence returns TICKING


    In summary, it is similar to Fallback, but it calls all underlying states together (in order), and not one-by-one.
    ConcurrentFallback waits until all underlying states have finished ( a "join")

    ```mermaid
    stateDiagram-v2     
        direction TB       
        classDef successClass  fill:darkgreen,color:white
        classDef tickingClass  fill:yellow,color:black
        classDef otherClass  fill:darkorange,color:white
        classDef abortClass  fill:darkred,color:white

        state fork_state <<fork>>    
        [*] --> fork_state
        fork_state --> state_1  
        fork_state --> state_2
        state join_state <<fork>>
        state_1 --> join_state : CANCEL    
        state_1 --> OTHER : OTHER outcome
        state_2 --> join_state : CANCEL
        state_2 --> OTHER : OTHER outcome
        join_state --> CANCEL
        state_1 --> TICKING : TICKING
        state_2 --> TICKING : TICKING
        

        state "TICKING <br> if any TICK transition <br> is received" as TICKING
        state "OTHER <br>returns first other outome" as OTHER
        state "CANCEL <br>if both transitions <br> are received" as CANCEL
        class SUCCEED successClass
        class OTHER otherClass
        class TICKING tickingClass
        class CANCEL abortClass
    ```
        
    """
    def __init__(self, name:str, 
                 children: List[TickingState]=[] ) -> None:
        """
        parameters:
            name: 
                name of the sequence
            children:
                a list of states 
                you can use add_state(...) to add children.

        """
        super().__init__(name,[],children)
        

   

    def co_execute(self,blackboard:Blackboard):
        """
        executes the underlying states in the fallback node, as much as possible concurrently.
        all outcomes except for CANCEL and TICKING indicate success.
        Fails with CANCEL when all underlying states have returned CANCEL.

        parameters:
            blackboard:
        """

        for s in self.states:
            s["active"] = True
        countActive = len(self.states)
        while (countActive > 0):
            ticked = False
            for s in self.states:
                if s["active"]==True:
                    outcome = s["state"](blackboard)
                    if outcome==TICKING:
                        ticked=True
                    elif outcome==CANCEL:
                        countActive=countActive-1
                        s["active"]=False
                    else:
                        # in case any of the underlying states was not completed:
                        for s in self.states:
                            s["state"].reset()
                        yield outcome  # yielded before any tick before or after in the sequence
            if ticked:
                yield TICKING
        # all of the underlying states have necessarily completed
        yield CANCEL
        

class WaitFor(Generator):
    """
    A state that delays until a condition is satisfied.

    ```mermaid
        stateDiagram-v2
            direction LR
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white

            state "WaitFor" as waitfor
            [*] --> waitfor
            waitfor --> TICKING : condition_cb()==False
            waitfor --> SUCCEED : condition_cb()==True
        
            class SUCCEED successClass
            class TICKING tickingClass
            
    ```
    """
    def __init__(self, condition_cb:Callable):
        """
        Ticks until condition is True and returns SUCCEED.

        Parameters:
            condition_cb:
                callback function with signature `condition(blackboard:Blackboard) -> bool`

        """
        outcomes = [SUCCEED]
        super().__init__("WaitFor",outcomes)
        self.condition_cb = condition_cb

    def co_execute(self,blackboard):
        while not self.condition_cb(blackboard):
            yield TICKING
        yield SUCCEED





class WaitForever(Generator):
    """
    A state that waits forever (while yielding TICKING)

    """
    def __init__(self):
        """
        Ticks forever, yielding TICKING

        """
        outcomes = []
        super().__init__("WaitForever",outcomes)

    def co_execute(self,blackboard):
        while True:
            yield TICKING


class GeneratorWithState(Generator):
    """
    A Generator with one underlying states
    """
    def __init__(self, name: str, outcomes: List[str], state: TickingState):
        """
        Parameters:
            name:
                name of the node
            outcomes:
                a list of strings indicating the expected outcomes, the outcomes of the underlying
                state are automatically added.        
            state:
                underlying state
        """
        if state is not None:
            outcomes = cleanup_outcomes(outcomes + state.get_outcomes())
        if not isinstance(state,TickingState):
            raise ValueError(f"{name} : state should be an instance of a subclass of State")
        if state.parent is not None:
            raise ValueError(f"{state.name} already belongs to {state.parent.name}")
        state.parent = self
 
        super().__init__(name,outcomes)
        self.state=state


    def accept(self, visitor:Visitor):
        if visitor.pre(self):
            self.state.accept(visitor)
        visitor.post(self)    

    def reset(self):  # general rule, if you own states, you have to reset thems
        if self.state is not None and isinstance(self.state,TickingState):
            self.state.reset()
        else:
            get_logger().info(f"GeneratorWithState, state was not reset, type {type(self.state)}")
        super().reset()  
    

class While(GeneratorWithState):
    """
    State that contiuously evaluates an underlying state as long as a condition is satisfied.
    ```mermaid
        stateDiagram-v2
            direction LR
            classDef successClass  fill:darkgreen,color:white
            classDef tickingClass  fill:yellow,color:black
            classDef otherClass  fill:darkorange,color:white
            classDef abortClass  fill:darkred,color:white


            state ConditionWhile {
                direction TB
                [*] --> my_state
                state "State" as my_state
            }
            [*] --> ConditionWhile
            my_state --> OTHER   : other outcome
            my_state --> TICKING : ticking<br>condition_cb()==True
            my_state --> CANCEL : condition_cb()==False
            class OTHER otherClass
            class TICKING tickingClass
            class CANCEL abortClass
    ```     
    """
    def __init__(self, name:str,condition_cb:Callable, state:TickingState):
        """
        Same behavior as the underlying state `state`, but returns CANCEL if condition_cb(self,condition_cb)
        ever gets false. 


        Parameters:
            name:
                name of the instance
            condition_cb:
                callback function with signature `condition(blackboard:Blackboard) -> bool`
            state:
                at each tick of the underlying state, the condition_cb is checked 
        """
        outcomes = [CANCEL]
        outcomes.append(CANCEL)  # Generator will add TICKING and ABORT
        super().__init__(name,outcomes,state)
        self.condition_cb = condition_cb
    
    def co_execute(self,blackboard):        
        while self.condition_cb(blackboard):
            outcome = self.state(blackboard)
            yield outcome
        yield CANCEL


class Repeat(GeneratorWithState):
    """
    Repeats underlying state for *maxcount* times or until an outcome other than SUCCEED is reached.
    ```mermaid
    stateDiagram-v2
        direction LR
        classDef successClass  fill:darkgreen,color:white
        classDef tickingClass  fill:yellow,color:black
        classDef otherClass  fill:darkorange,color:white


        state Repeat {
            [*] --> my_state
            state "State" as my_state
            my_state --> my_state : succeed
        }
        [*] --> Repeat
        my_state --> SUCCEED : #succeed > maxcount
        my_state --> OTHER   : other outcome
        my_state --> TICKING : ticking
        class SUCCEED successClass
        class OTHER otherClass
        class TICKING tickingClass
        
    ``` 
    Note:
        There is no tick between the repetitions.  If you have an underlying state that directly returns
        SUCCEED, the sequence will be executed without ticks.

    """
    def __init__(self,name:str, maxcount:int, state: TickingState):
        """Repeats underlying state for *maxcount* times or until an outcome other than SUCCEED is reached

        Parameters:
            name:   
                name of the instance
            maxcount: 
                Maximum number of times that underlying state will be executed. -1 indicates that you
                want to loop forever.
            state:
                underlying state
        """         
        super().__init__(name,[],state)
        self.maxcount = maxcount
        
    def co_execute(self,blackboard): 
        if self.maxcount!=-1:       
            for c in range(self.maxcount):
                outcome = self.state(blackboard)
                while outcome==TICKING:
                    yield TICKING
                    outcome = self.state(blackboard)
                if outcome!=SUCCEED:
                    yield outcome
            yield SUCCEED
        else:
            # forever:
            while True:
                outcome = self.state(blackboard)
                if outcome!=SUCCEED:
                    yield outcome


class Message(Generator):
    """
    Message(msg="my_message") returns a State that displays a message
    Message(cb=function) returns a State that displays a message generated by the given function
    The argument name="..." specifies the name of the Message state.
    """

    _name_counter=0

    def __init__(self,name:str=None,*,msg:str=None, cb:Callable=None, logCategory:str="default") -> None:
        """
        Displays a message to the log. 

        Parameters:
            name:
                instance name, default `message`
            msg:
                string describing the message
            cb:
                callback returning the message.  Signature `def cb(blackboard)->str`.
                The callback allows the user to compute the message at time of evaluation,
                e.g. to report on values on the blackboard.
            logCategory:
                the category under which to log the message
                
        warning:
            Only one of the arguments msg or cb can be specified

        example:
            ```
            Message(lambda bb: f'{bb["some_key_in_blackboard"]=}' )
            ```
        """
        if name is None:
            Message._name_counter +=1
            name = f"message_{Message._name_counter}"
        super().__init__(name,[SUCCEED,])
        if msg is None and cb is None: raise ValueError("Message : you should specify a message (did you only specify a name?)\nTip:  Message(msg='my message text') or Message('message_name','message text')  ") 
        self.msg = msg
        self.cb  = cb
        self.logcat = logCategory
        if not(self.msg is None) and not(self.cb is None) and not(self.msg is None and self.cb is None):
            raise ValueError("Message: you have to specify exactly one of the msg or cb arguments in the constructor")
        
    def co_execute(self,blackboard: Blackboard):
        #log = my_node.get_logger()
        #log.info(f'Entering MyMessage : {self.msg}')
        if self.cb is not None:
            get_logger(self.logcat).info( self.cb(blackboard) )
        elif self.msg is not None:
            get_logger(self.logcat).info( self.msg )
        yield SUCCEED


def dumps_blackboard(blackboard, indent_spaces: int = 4) -> str:
    """
    Recursively constructs a  formatted, indented string 
    representing a hierarchical Python data structure.
    """
    return json.dumps(blackboard,indent=indent_spaces,skipkeys=True,check_circular=True,default=lambda ob: ob.__repr__())

# def dumps_blackboard(blackboard:Blackboard,indent:int=0):
#     """
#     returns a string-dump of a (piece of the ) blackboard
    
#     Parameters:
#         blackboard:
#             Blackboard to be dumped
#         indent:
#             determines the indentation for printing.
#     """
#     s = ""
#     indent += 4
#     space = f'{" ":{indent}}'
#     if isinstance(blackboard,bool):
#         s = f'{blackboard}\n'
#     elif isinstance(blackboard,int):
#         s = f'{blackboard}\n'
#     elif isinstance(blackboard,float):
#         s = f'{blackboard}\n'    
#     elif isinstance(blackboard,dict):
#         s = s + "\n"
#         for k,v in blackboard.items():
#             s+= space + k + " : " + dumps_blackboard(v,indent)
#     elif isinstance(blackboard, list):
#         listofnumbers=True
#         first = True
#         for item in blackboard:
#             if isinstance(item,int) or isinstance(item,float):
#                 listofnumbers=True
#                 break
#             if first:
#                 s = s + "\n"
#                 first=False
#             s+= space + dumps_blackboard(item,indent)
#         if listofnumbers:
#             s = f'{blackboard}\n'                 
#     elif isinstance(blackboard,str):
#         s = f'"{blackboard}"\n'       
#     elif isinstance(blackboard,list):
#         s = f'{blackboard}\n'             
#     else:
#         s = f'{type(blackboard)}'
#     return s


class LogBlackboard(Generator):
    """
    Logs blackboard or part of blackboard
    """
    def __init__(self, name:str, location:List[str]=[]) -> None:
        """
        Prints (a part of) the blackboard to the log.  Info-level is used.

        Parameters:
            name:
                instance name
            location:
                a list of strings that describes a location in the blackboard.
        """
        super().__init__(name,[SUCCEED])
        self.location = location

    def co_execute(self,blackboard: Blackboard):
        bb = blackboard
        for k in self.location:
            bb = bb[k]

        
        get_logger().info("Blackboard:\n"+dumps_blackboard(bb) )
        yield SUCCEED




class Compute(Generator):
    def __init__(self, name:str, location:List[str], cb:Callable):
        """
        State that performs some computations using a callback function
        and places the results in a predetermined location in the blackboard.

        Parameters:
            name:
                name of this node
            location:
                location in the blackboard, given as a list of strings.  Should point to
                a dictionary.
            cb:
                callback function with signature `def cb(blackboard)->dict`. The dictionary 
                contents will be inserted in the given location 
        """
        super().__init__(name,[SUCCEED])
        self.location = location
        self.cb = cb
    def co_execute(self,blackboard):
        bb = blackboard
        for k in self.location:
            if k not in bb:                
                bb[k]={}
            bb = bb[k]
            if not isinstance(bb,dict):
                raise ValueError("location should point to a dictionary in the blackboard")
        bb.update( self.cb(blackboard) )
        yield SUCCEED


class Adapt(GeneratorWithState):
    def __init__(self,name:str, state:TickingState, outcomeMap:Dict[str,str]=[])->None:
        """
        Adapts the outcome of a state using a transition table

        Parameters:
            name:
                instance name
            state:
                state to be adapted
            outcomeMap:
                a dictionary that maps outcomes of state to their new values.  If the outcome
                is not present in the dictionary, it is returned unchanged.
        """        
        super().__init__(name,[],state)
        self.outcomes = [ e for e in state.get_outcomes() if e not in outcomeMap ] + [ v for e,v in outcomeMap.items()]
        self.outcomeMap = outcomeMap
        
    def co_execute(self,blackboard:Blackboard):
        while True:
            out = self.state(blackboard)
            out = self.outcomeMap.get(out,out)
            yield out

class AlwaysOutcome(TickingState):
    def __init__(self,name:str,outcome:str=None) -> None:
        """
        Returns a TickingState that always returns the same outcome.

        Parameters:
            name(str): 
                name of the Ticking state
            outcome(str):
                outcome that will be returned (typically, but not necessarily: SUCCEED, CANCEL)
        """
        if outcome is None:
            outcome = name
            name = None
        super().__init__(name,[outcome,])        
        self.outcome = outcome
    def entry(self, blackboard):
        return self.outcome


# class AlwaysOutcome(Generator):
#     def __init__(self,name:str,outcome:str=None) -> None:
#         """
#         Returns a TickingState that always returns the same outcome.

#         Parameters:
#             name(str): 
#                 name of the Ticking state
#             outcome(str):
#                 outcome that will be returned (typically, but not necessarily: SUCCEED, CANCEL)
#         """
#         if outcome is None:
#             outcome = name
#             name = "always_outcome"
#         super().__init__(name,[outcome,])
#         assert(outcome != TICKING)
#         self.outcome = outcome
#     def co_execute(self,blackboard):
#         while True:
#             yield self.outcome


class TimedWait(Generator):
    """Node that waits for a given time and then returns succeed

    This is the plain python version, there also exists a ROS2 version.

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
    def __init__(self,name:str,timeout:float =1.0 ):
        """
        TimedWait waits for a given time and then returns succeed.

        Parameters:
            name:
                instance name
            timeout:
                duration to wait.

        will return TICKING until timeout is passed after which it returns SUCCEED
        """
        outcomes = [SUCCEED,ABORT]
        super().__init__(name,outcomes)
        self.timeout = timeout
    
    def co_execute(self,blackboard):     
        get_logger().info(f"{self.name} : waiting for {self.timeout}")
        start_time = time.monotonic()
        while (time.monotonic()-start_time) < self.timeout:
            yield TICKING
        get_logger().info(f"{self.name} : finished waiting")
        yield SUCCEED


class TimedRepeat(GeneratorWithState):
    """
    Repeats an underlying state for a given number of times and a given time interval.

    This is the pure python version of this class, there also exists a ROS2 version.

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
            timeout: float,
            state: TickingState
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

        Note:
            if the underlying state returns later than `timeout` with a non-ticking outcome, an exception will be raised 
            and abort is called.
        """
        super().__init__(name,[SUCCEED],state)
        self.maxcount = maxcount
        self.timeout = timeout
        self.log  = get_logger()
    
    def co_execute(self,blackboard):        
        starttime = time.monotonic()
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
            current_time = time.monotonic()
            if current_time >= 2*looptime:
                raise Exception("Loop time is more than 2 times exceeded by underlying state")
            # tick until next timeout interval            
            while current_time < looptime:
                yield TICKING
                current_time = time.monotonic()
            looptime = looptime + self.timeout
            count    = count + 1
        yield SUCCEED
    


# class StateMachineElement:
#     """
#     Just to have a type that a visitor could recognize
#     """
#     def __init__(self,name,state,transitions):
#         self.name = name
#         self.state = state
#         self.transitions = transitions

#     def accept(self, visitor: Visitor) :
#         if visitor.pre(self):
#             self.state.accept(visitor)
#         visitor.post(self)


# def default_transitioncb(statemachine,blackboard,source,outcome):
#     """
#     Callback for use in cbStateMachine

#     Parameters:
#         statemachine: 
#             statemachine in which this callback is called
#         blackboard: 
#             the blackboard wich was used to execute this statemachine
#         source: 
#             the source state of the transition
#         outcome: 
#             the name of the transition

#     Returns:
#         outcome or an override of the outcome
#     """
#     return outcome

# def default_statecb(statemachine,blackboard,state):
#     """
#     Default callback used in TickingStateMachine.

#     Parameters:
#         statemachine:
#             statemachine in which this callback is called
#         blackboard:
#             the blackboard wich was used to execute this statemachine
#         state:
#             state that will be entered
#     """
#     pass


# may 2026: removed transition_cb and statecb:
class TickingStateMachine(TickingState):
    """
    A StateMachine that calls a callback function before entering a state and/or at each transition.
    
         
    This statemachine is capable of working together with TickingState:

    - will exit when TICKING outcome is given by one of the substates, but then if it is called again,
      it will have remembered the state that had the TICKING outcome and start from that state.
    - if returning with any other outcome, will start next time from the start state.
        
    """ 

    #, transitioncb=default_transitioncb, statecb=default_statecb) -> None:   
    def __init__(self, name:str, outcomes: List[str]) -> None:         
        """
        TickintStatemachine is a statemachine that can maintain TickingStates.

        Parameters:
            name: 
                (instance) name of the state machine
            outcomes: 
                the allowed outcomes of the state machine, any outcome not specified in transitions
                will be an outcome of the state machine and should be contained in outcomes (otherwise exception+abort)         
        """
        super().__init__(name,outcomes)

        self.states = {}
        self.states_ordered = [] # to keep track of the order of declaration, for visualization.
        self.start_state = None
        self.current_state = None
        self.default_transitions = {}
        
    def set_default_transitions(self,transitions: Dict[str, str|TickingState] = {}):
        """ 
        Adds default transitions to each state added afterwards. The transitions
        specified in `add_state` add to or overwrite these default transitions.

        Parameters:
            transitions:
                a dictionary that maps outcomes of the state to other outcomes or another state. See `add_state`

        """
        for k,v in transitions.items():
                    if not isinstance(k,str):
                        raise Exception("TickingStateMachine.add_state() the key of the transitions dictionary should be a string")
                    if isinstance(v,TickingState):
                        transitions[k] = v.name 
                    elif not isinstance(v,str):
                        raise Exception("TickingStateMachine.add_state() the value of the transitions dictionary should be a string or a derived class from TickingState")
        self.default_transitions = transitions

        
    def add_state(
        self,
        state: TickingState,
        transitions: Dict[str, str|TickingState] = None
    ) -> None:
        """
        add_state(state,transitions) adds a state and associates transitions in this state-machine
        with this state.  By default to first state added will be the start state. This can be changed
        using the method `set_start_state`.

        Parameters:
            state: 
                state to be added, it will be added under its name (i.e. state.name, as defined in TickingState)
            transitions:
                A Python Dict that maps outcome of the state to a target (see note below).
                - a dictionary that maps outcomes of the state to names of a state in this state machine.
                - As a shortcut/alternative, this can also be a dictionary that maps outcomes to TickingState 
                (in that case TickingState.name is used)

        Note:
          - The keys of the transitions Dict are outcomes. The values are targets.
          - A target can be a **state object** or the  **name of a state**.
          - A target can also be an **outcome** that is listed in the outcomes of the state machine. In that case the 
            state machine will **exit** with that outcome.
          - A target can be another outcome that the statemachine will try to resolve.  This is mostly useful for
            default transitions (see `set_default_transitions`).  This redirection can even redirect 
            TICKING (for advanced use).
          - a BeTFSM object can only be added as a child once.  BeTFSM hierarchy is required to be a tree,
            not a more general graph.                
        """
        if not isinstance(state,TickingState):
            raise Exception("TickingStateMachine.add_state() only accepts states that are subclasses of TickingState")
        if state.parent is not None:
            raise ValueError(f"{state.name} already belongs to {state.parent.name}")                        
        if transitions is None:
            transitions = {}
        if not isinstance( transitions , Dict):
            raise ValueError("transitions should be a dictionary")             
        transitions_unified = self.default_transitions.copy()  # should be shallow copy  but not reference     
        for k,v in transitions.items():
            if not isinstance(k,str):
                raise Exception("TickingStateMachine.add_state() the key of the transitions dictionary should be a string")
            if isinstance(v,TickingState):
                transitions_unified[k] = v.name 
            elif not isinstance(v,str): 
                raise Exception("TickingStateMachine.add_state() the value of the transitions dictionary should be a string or a derived class from TickingState")
            else:
                transitions_unified[k] = v  # add or overwrite the default transitions
        if state.name in self.states:
            raise ValueError("state has a name that is already known to the state machine: duplicate name or state has been added twice")
        self.states[state.name] = {
            "state": state,
            "transitions": transitions_unified
        }                
        state.parent = self
        self.states_ordered.append(state)        
        if self.start_state is None:
            self.start_state = state.name
            self.current_state = state.name
        
        
    def set_start_state(self, name: str|TickingState) -> None:
        """
        Explicitly states the starting state. States are specified using their name

        Parameters:
            name:
                set the state by which the state machine starts.  By default the first state 
                added. Can also be an instance of TickingState descendant, in that case this
                is the same as name.name
        """
        if isinstance(name,TickingState):
            name = name.name
        self.start_state = name
        self.current_state = name

    def get_start_state(self) -> str:
        return self.start_state

    def reset(self):
        """
        Resets the state-machine.  Ensures that the next call will start again
        from the starting state.  Calls reset on all the states it contains.
        """
        for s,v in self.states.items():
            v["state"].reset()
        self.current_state = self.start_state
        super().reset()

    def accept(self, visitor: Visitor):
        """
        accepts a visitor to go through all states of an hierarchy
        """
        if visitor.pre(self):
            for k,v in self.states.items():
                v["state"].accept(visitor)
        visitor.post(self)

    def entry(self, blackboard: Blackboard) -> str:
        self.current_state = self.start_state
        return CONTINUE

    def doo(self, blackboard: Blackboard) -> str:
        while True:
            state = self.states[self.current_state]
            name = self.current_state
            outcome = state["state"](blackboard)         
            # translate outcome using transitions
            if outcome in state["transitions"]:              
                outcome = state["transitions"][outcome]
            if outcome in state["transitions"]:           # in case outcome points to a reference of another outcome      
                outcome = state["transitions"][outcome]                
            if outcome == TICKING:                # outcome is TICKING and exits state machine but keeps current state                                                 
                return outcome
            elif outcome in self.get_outcomes():  # outcome is an outcome of the sm, reset current state
                return outcome 
            elif outcome in self.states:          # outcome is a state
                self.current_state = outcome
            else:                                 # outcome is not in the sm
                raise Exception(f"""State {name} has outcome ({outcome}) without transition specified
                                    transitions defined for state {state['transitions']} 
                                    outcomes of state machine {self.get_outcomes()} 
                                    states of state machine   {[s.name for s in self.states_ordered]}
                                 """)
    
    def exit(self) -> str:
        self.current_state = self.start_state
        return super().exit()
 


def blackboard_polling_func( path:str ) -> Callable[[Dict, List[str]],str|None]:
    """
    Returns a polling function that checks whether one of the given events
    are present in the blackboard at location `path`.

    Arguments:
        path:
            location in the blackboard as a '/' separated path.

    Returns:
        polling function

    """
    def polling(bb, events):
        ev = get_path_value(path)
        if ev is not None and ev in set(events):
            set_path_value(bb,path,"")  # you need to 'consume' the event !!
            return str(ev)
        return None
    return polling

def ctrl_c_polling_func(value:str="CTRL_C", repeated=3 ) -> Callable[[Dict, List[str]],str|None]:
    """
    Returns a polling function that checks whether one of the given events
    are present in the blackboard at location `path`.

    Configures the system SIGINT (Ctrl+C) handler to interact with a blackboard, where
    a variable will set to given value if Ctrl-c is pressed

    Arguments:
        value (str):
            Value that will be set when ctrl-c is pressed.
        repeated (int, None): 
            number of times to repeat ctrl-c before KeyboardInterrupt is raised

    Returns:
        polling function

    """
    receiver = Ctrl_C_Receiver.get_instance(value, repeated)
    def polling(bb, events):
        return receiver.poll_for(events)
    return polling


def  combine( *args):
    """    
    Combines multiple polling callbacks, takes as arguments the functions themselves
    and returns a callback function that combines the polling functions.

    e.g. combine(polling_func1,polling_func2,....)

    The earlier arguments have priority
   
    """
    def polling(cb, events):
        for e in args:
            ev = e(cb,events)
            if ev is not None:
                return ev
        return None
    return polling


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
        the event, i.e. sets the value in blackboard to ""
        """          
        if self.buffer in target_strings:
            retval = self.buffer
            self.buffer = ""
            return retval
        else:
            return None



class EventOutcome(Generator):
    """
    !!! warning
        Preliminary - untested version

    Same as Event, but no subtrees in the event_map, only outcomes.
    Is easier to understand.
    """
    def __init__(self, 
                 name:str, 
                 event_poller:Callable[[Dict, List[str]],str|None], 
                 event_map=Dict[str, str] ):
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
        while True:
            ev = self.event_poller(blackboard, self.events )
            if ev is None:
                ev = NO_EVENT
            outcome = self.event_map[ev]
            if outcome!=TICKING:
                break
            yield TICKING
        yield outcome
        


class EventConcurrent(GeneratorWithList):
    """    
    !!! warning
        Preliminary - untested version

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
                 event_poller:Callable[[Dict, List[str]],str|None], 
                 event_map=Dict[str, TickingState] ):         
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
        countActive = 0
        nominal_succeeded = False
        while True:                    
            # poll events related to non-active states.
            while True:
                state_events = [  k for k,v in self.event_to_state_ndx.items() if self.states[v]["active"]==False ]
                ev = self.event_poller(blackboard, state_events )
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

        while True:
            # consume all allowable events  (less efficient with many matching events, no choice due to API)
            while True:
                ev = self.event_poller(blackboard, state_events )
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
        print(f"{cls._instances=}")
        print(f"{channel_name=}")
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



def http_polling_func(        
        channel_name:str="betfsm",
        queue_size:int=10, 
        max_age:float=0.5
    ) -> Callable[[Dict, List[str]],str|None]:       
    """ 
    Returns a callback function to read HTTP events suitable for use withy betfsm.Event.
    
    Parameters:
        channel_name:
            name of the topic to subscribe to

        queue_size:
            size of the queue related to maximum concurrent events, i.e. sample time in relation
            to the events generated. Because the underlying receiver is a singleton, the maximum
            queue size of everybody who requested an instance of the receiver is taken.

        max_age:
            maximum age of the events that still will be received.
            
    Returns:
        Polling function
    """     
    print(f"http_polling_func({channel_name=}{queue_size=}{max_age=} )")
    sub = HTTPEventReceiver.get_instance(channel_name,queue_size)    
    def polling(bb, events):
        return sub.poll_recent_for(events,max_age)
    return polling

