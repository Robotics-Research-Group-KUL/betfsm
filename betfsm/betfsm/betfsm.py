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

#
# Draft


# from simple_node import Node


from typing import Dict, List, Union, Callable,Type, TypeAlias
from enum import Enum
from threading import Lock
from abc import ABC, abstractmethod
import traceback

import time
from .logger import get_logger



Blackboard: TypeAlias = Dict[str, Dict|any]


def cleanup_outcomes(outcomes:List[str])->List[str]:
    """
    cleans up a list of outcomes by elliminating duplicates.

    Parameters:
        list of outcomes with possibly duplicate elements.

    Returns:
        list of outcomes with duplicate elements elliminated.
    """
    return [ e for e in {e for e in outcomes}]


#
# outcomes with some special meaning for some TickingStates
# just to allow a more systematic definition of states
#

SUCCEED  = "succeeded"    # everything is fine, continue as normal
ABORT    = "aborted"      # involuntary stop, e.g. due to exception raised, communication failure,...
CANCEL   = "canceled"     # voluntary stop, deliberatly provoked, e.g. reacting to cancel request of an action
TIMEOUT  = "timeout"      # some operation times out.
TICKING  = "ticking"      # only use this to yield and expecting to be called back the next tick
CONTINUE = "continue"     # only used in the entry() method of TickingState, to signal tht you want to directly continue with Doo().
                          # don't use it anywhere else


TickingState_Status = Enum("TickingState_Status",["ENTRY","DOO","EXIT"])
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
    def __init__(self,name:str, outcomes: List[str]):
        """
        parameters:
            name:
                name of the TickingState.  This is meant to be an instance-name, not a class-name.
            outcomes:
                all possible outcomes of the state, TICKING and ABORT will be added.
        """
        self.outcomes = outcomes
        self.outcomes.append(TICKING)
        self.outcomes.append(ABORT)
        self.name = name
        self.status = TickingState_Status.ENTRY
        self.outcome = "" # will contain the last used outcome


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
        exit() is called when appropriate
        """
        if self.status == TickingState_Status.DOO:
            self.exit()
        self.status = TickingState_Status.ENTRY

    def execute(self, blackboard: Blackboard) -> str:        
        #self.log.info("TickintState.execute")
        if self.status == TickingState_Status.ENTRY: 
            get_logger("state").info(f"Entering {self.name}")
            try:
                self.outcome = self.entry(blackboard)
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
            except Exception as e:
                get_logger().error(f"{self.name} : exception occured : "+ traceback.format_exc())
                self.outcome = ABORT
                self.status = TickingState_Status.EXIT
            if self.outcome == TICKING:
                return TICKING
            self.status = TickingState_Status.EXIT

        if self.status == TickingState_Status.EXIT:
            self.outcome = self.exit()
            self.status = TickingState_Status.ENTRY
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
        while True:
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

        

    def __str__(self) -> str:
        pass


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
            raise Exception("add_state expects as second argument an instance of a subclass of State")
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
    until a child returns an outcome different from TICKING.

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


    def co_execute(self,blackboard):
        """        
        Python generator routine (co-routine). You can use `yield <outcome>` to return intermediate results

        Parameters:
            blackboard:
        """
        while True:
            for s in self.states:
                outcome = s["state"](blackboard)
                if outcome!=TICKING:
                    yield outcome
            yield TICKING






class Fallback(GeneratorWithList):
    """    
    Implements a behaviortree-like Fallback node:
      - success any other outcome,
      - failure is CANCEL outcome

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
            Loop --> CANCEL : last state succeeded
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
    def __init__(self, name:str, children: List[TickingState]=[]) -> None:
        """
        parameters:
            name: 
                name of the sequence
            children:
                a list of states 
                you can use add_state(...) to add children.
        """
        super().__init__(name,[], children)
        self.active_state = []
        

    def co_execute(self,blackboard):
        for s in self.states:             
            self.active_state = [s["name"]]
            outcome = s["state"](blackboard)
            while outcome==TICKING:
                yield TICKING
                outcome = s["state"](blackboard)
            if outcome!=CANCEL:
                yield outcome                            
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
       the Sequence returns CANCEL (a "success")
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
    def __init__(self, condition_cb):
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

    def __init__(self,name:str=None,*,msg:str=None, cb:Callable=None, logFunc=None) -> None:
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
            logFunc:
                function to call for logging the message, by default get_logger.info()
                
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
        self.msg = msg
        self.cb  = cb
        if logFunc is None:
            self.log = get_logger().info
        else:
            self.log = logFunc
        if not(self.msg is None) and not(self.cb is None) and not(self.msg is None and self.cb is None):
            raise ValueError("Message: you have to specify exactly one of the msg or cb arguments in the constructor")
        
    def co_execute(self,blackboard: Blackboard):
        #log = my_node.get_logger()
        #log.info(f'Entering MyMessage : {self.msg}')
        if self.msg is not None:
            self.log(self.msg)
        if self.cb is not None:
            self.log(self.cb(blackboard))
        yield SUCCEED


def dumps_blackboard(blackboard,indent=0):
    """
    returns a string-dump of a (piece of the ) blackboard
    
    Parameters:
        blackboard:
        indent:
            determines the indentation for printing.
    """
    s = ""
    indent += 4
    space = f'{" ":{indent}}'
    if isinstance(blackboard,bool):
        s = f'{blackboard}\n'
    elif isinstance(blackboard,int):
        s = f'{blackboard}\n'
    elif isinstance(blackboard,float):
        s = f'{blackboard}\n'    
    elif isinstance(blackboard,dict):
        s = s + "\n"
        for k,v in blackboard.items():
            s+= space + k + " : " + dumps_blackboard(v,indent)
    elif isinstance(blackboard, list):
        listofnumbers=True
        first = True
        for item in blackboard:
            if isinstance(item,int) or isinstance(item,float):
                listofnumbers=True
                break
            if first:
                s = s + "\n"
                first=False
            s+= space + dumps_blackboard(item,indent)
        if listofnumbers:
            s = f'{blackboard}\n'                 
    elif isinstance(blackboard,str):
        s = f'"{blackboard}"\n'       
    elif isinstance(blackboard,list):
        s = f'{blackboard}\n'             
    else:
        s = f'{type(blackboard)}'
    return s


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
    def __init__(self,name:str, state:TickingState, transitions=[])->None:
        """
        Adapts the outcome of a state using a transition table

        Parameters:
            name:
                instance name
            state:
                state to be adapted
            transitions:
                a dictionary that maps outcomes of state to their new values.  If the outcome
                is not present in the dictionary, it is returned unchanged.
        """        
        super().__init__(name,[],state)
        self.outcomes = [ e for e in state.get_outcomes() if e not in transitions ] + [ v for e,v in transitions.items()]
        self.transitions = transitions
        
    def co_execute(self,blackboard:Blackboard):
        while True:
            out = self.state(blackboard)
            new_out = self.transitions.get(out)
            if new_out is None:
                new_out = out
            yield out

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

        Returns:
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


def default_transitioncb(statemachine,blackboard,source,outcome):
    """
    Callback for use in cbStateMachine

    Parameters:
        statemachine: 
            statemachine in which this callback is called
        blackboard: 
            the blackboard wich was used to execute this statemachine
        source: 
            the source state of the transition
        outcome: 
            the name of the transition

    Returns:
        outcome or an override of the outcome
    """
    return outcome

def default_statecb(statemachine,blackboard,state):
    """
    Default callback used in TickingStateMachine.

    Parameters:
        statemachine:
            statemachine in which this callback is called
        blackboard:
            the blackboard wich was used to execute this statemachine
        state:
            state that will be entered
    """
    pass


class TickingStateMachine(TickingState):
    """
    A StateMachine that calls a callback function before entering a state and/or at each transition.
    
         
    This statemachine is capable of working together with TickingState:

    - will exit when TICKING outcome is given by one of the substates, but then if it is called again,
      it will have remembered the state that had the TICKING outcome and start from that state.
      - if returning with any other outcome, will start next time from the start state.
    - should be drop in replacement of Yasmin StateMachine, (but not the other way around, StateMachine can't
       handle TickingStates
        
    """    
    def __init__(self, name:str, outcomes: List[str], transitioncb=default_transitioncb, statecb=default_statecb) -> None:
        """
        TickintStatemachine is a statemachine that can maintain TickingStates.

        Parameters:
            name: 
                (instance) name of the state machine
            outcomes: 
                the allowed outcomes of the state machine, any outcome not specified in transitions
                will be an outcome of the state machine and should be contained in outcomes (otherwise exception+abort) 
            transitioncb: 
                callback function called at each transition. 
                Signature transitioncb(source_state:str, transtion:str, target_state:str).
                See also 
            statecb: callback function called before entering each state. 
                    Signature statecb(name)        
        """
        outcomes.append(TICKING)
        super().__init__(name,outcomes)

        self.states = {}
        self.start_state = None
        self.current_state = None
        self.statecb = statecb
        self.transitioncb = transitioncb
        
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
                a dictionary that maps outcomes of the state to names of a state in this state machine.
                As a shortcut/alternative, this can also be a dictionary that maps outcomes to TickingState 
                (in that case TickingState.name is used)
        """
        if not isinstance(state,TickingState):
            raise Exception("TickingStateMachine.add_state() only accepts states that are subclasses of TickingState")
        if transitions is None:
            transitions = {}
        if not isinstance( transitions , Dict):
            raise ValueError("transitions should be a dictionary")            
        for k,v in transitions.items():
            if not isinstance(k,str):
                raise Exception("TickingStateMachine.add_state() the key of the transitions dictionary should be a string")
            if isinstance(v,TickingState):
                transitions[k] = v.name 
            elif not isinstance(v,str):
                raise Exception("TickingStateMachine.add_state() the value of the transitions dictionary should be a string or a derived class from TickingState")
        self.states[state.name] = {
            "state": state,
            "transitions": transitions
        }
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
            self.statecb(self,blackboard,name)
            outcome = state["state"](blackboard)         
            outcome = self.transitioncb(self,blackboard,self.current_state, outcome)
            # translate outcome using transitions
            if outcome in state["transitions"]:              
                outcome = state["transitions"][outcome]
            if outcome == TICKING:                # outcome is TICKING and exits state machine but keeps current state                                                 
                return outcome
            elif outcome in self.get_outcomes():  # outcome is an outcome of the sm, reset current state
                return outcome 
            elif outcome in self.states:          # outcome is a state
                self.current_state = outcome
            else:                                 # outcome is not in the sm
                raise Exception(f"""State {name} has outcome ({outcome}) without transition specified\n
                                    transitions {state['transitions']} \n
                                    outcomes state machine {self.get_outcomes()} """)
    
    def exit(self) -> str:
        self.current_state = self.start_state
        return super().exit()
 
class BeTFSMRunner:
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.
    """
    def __init__(self, statemachine: TickingStateMachine, blackboard: Blackboard, frequency: float=100.0):
        """
        Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
        runs in the main thread.

        Parameters:
            statemachine:
                the TickingStateMachine to be run
            blackboard:
                the blackboard to be used
            frequency:
                frequency at which the statemachine is ticked (in Hz)
        """
        self.statemachine = statemachine
        self.blackboard = blackboard
        self.frequency = frequency
        self.interval_sec = 1.0/frequency

#    def run(self):
#        """
#        Runs the statemachine until it returns an outcome different from TICKING
#        Returns:
#            the final outcome of the statemachine
#        """
#        import time
#        rate = 1.0 / self.frequency
#        outcome = self.statemachine(self.blackboard)
#        while outcome == TICKING:
#            start = time.time()
#            outcome = self.statemachine(self.blackboard)
#            elapsed = time.time() - start
#            sleep_time = rate - elapsed
#            if sleep_time > 0:
#                time.sleep(sleep_time)
#        return outcome
#       

    def run(self):
        """
        Runs the statemachine until it returns an outcome different from TICKING

        Refers to absolute time to avoid drifting. Uses monotonic clock to 
        avoid problems with system time changes.

        Returns:
            the final outcome of the statemachine
        """

        # Use monotonic clock to avoid issues with system time changes
        start    = time.monotonic()
        next_run = start + self.interval_sec
        outcome  = TICKING
        while outcome == TICKING:
            outcome = self.statemachine(self.blackboard)
            # Sleep until the next scheduled time
            now = time.monotonic()
            sleep_time = next_run - now

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, log drift
                get_logger().warn(f"[Warning] Drift detected: {abs(sleep_time):.3f}s late")

            # Schedule next run
            next_run += self.interval_sec

