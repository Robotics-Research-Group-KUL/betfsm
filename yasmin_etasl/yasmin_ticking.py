# proposed additions/adaptations of etasl_yasmin_utils by Erwin.
#
# Draft


# from simple_node import Node
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin.state import State
from typing import Dict, List, Union, Callable

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

#
# blackboard.task[taskname].parameters = dict with task parameters
#

#
# For now, this  is a copy of cb_state_machine.py from yasmin_action.
#
#
# def default_parameter_setter(blackboard:Blackboard, parameters:dict) -> dict:
#     """
#     default do-nothing callback function to set parameters.

#     Input
#     -----
#     - blackboard: the blackboard, in case additional information is needed to construct the parameters
#     - parameters: parameters as read from tasks-schema.json

#     Output:
#     -------
#     - updated parameters
#     """
#     return parameters

# class ReadTaskParametersCB(ServiceState):
#     """Instructs the eTaSL node to read in parameters for the given task with a callback function
    
#        could replace the original ReadTaskParameters without much compatibility issues.
#     """
#     def __init__(self, task_name: str, setparamcb: default_parameter_setter) -> None:
#         super().__init__(
#             TaskSpecificationString,  # srv type
#             "/etasl_node/readTaskSpecificationString",  # service name
#             self.create_request_handler,  # cb to create the request
#             [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
#             self.response_handler,  # cb to process the response
#             timeout = 2.5 #seconds 
#         )
#         self.task_name = task_name
#         self.setparamcb = setparamcb
#         # self.file_path = file_path

#     def create_request_handler(self, blackboard: Blackboard) -> TaskSpecificationString.Request:

#         req = TaskSpecificationString.Request()

#         task = get_task(self.task_name,blackboard)

#         # filter and copy task["parameters"]
#         param = {}
#         for key,value in task["parameters"].items():
#             if (key[:3]!="is-") and (key!="file_path"):
#                 param[key] = value
#         paramdef = param.copy()
#         param = self.setparamcb(blackboard, param)
#         param_string = ""
#         # parameter checking using paramdef
#         for key,value in param.items():
#             if key not in paramdef:
#                 raise ValueError(f"callback sets {key} parameter that is not in schema ")
#             if value == "external":
#                 raise ValueError(f"parameter declared 'external' is not set by callback")
#         for key,_ in paramdef.items():
#             if key not in param:
#                 raise ValueError(f"required parameter {key} is not defined" )
        
#         # lua script setting parameters:
#         for key, value in param.items():
#             if isinstance(value,bool):
#                 param_string = param_string + f"{key} = {str(value).lower()}\n"
#             else:
#                 param_string = param_string + f"{value}"

#         req.str = param_string

#         print(f"{param_string=}")
#         # print("ReadTaskSpecificationString")
#         return req

#     def response_handler(self,blackboard: Blackboard,response: TaskSpecificationString.Response) -> str:

#         # print("Service response success: " + str(response.success))
#         blackboard.success = response.success
#         if not response.success:
#             return ABORT
#         # time.sleep(1)
#         return SUCCEED


def default_transitioncb(statemachine,blackboard,source,outcome):
    """
    Callback for use in cbStateMachine

    Parameters
    ----------
    statemachine: statemachine in which this callback is called
    blackboard  : the blackboard wich was used to execute this statemachine
    source : the source state of the transition
    outcome: the name of the transition

    Returns
    -------
    outcome or an override of the outcome
    """
    return outcome

def default_statecb(statemachine,blackboard,state):
    pass


#SUCCEED = "succeeded"
#ABORT = "aborted"
#CANCEL = "canceled"
#TIMEOUT = "timeout"

TICKING="ticking"
CONTINUE="continue"
EXIT="exit"
# TIMEOUT="timout"  #already defined

TickingState_Status = Enum("TickingState_Status",["ENTRY","DOO","EXIT"])

"""
    ```graphviz

        digraph monitoringstate{
            //node [shape=point] start;      
            //node [shape=point] end;
            node [shape=point] start
            node [shape=box, style=rounded];
        

            node [label= "  entry()\n if TICK return TICK\n if exception return ABORT"] Entry;

            node [label= "  doo()\n if TICK return TICK\n if exception return ABORT"] Doo;
            node [label= "  exit()\n return outcome \n if exception return ABORT"] Exit;
            start->Entry

            Entry -> Doo [label="CONTINUE\nor TICK"]

            Entry -> Exit [label="≠TICK and\n ≠CONTINUE\nor ABORT"]
            Doo -> Exit [label="≠TICK\nor ABORT"]
            Doo -> Doo [label="TICK"]
            Exit -> Entry 
        
        }
    ```
"""

class TickingState(State):
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
    def __init__(self,outcomes: List[str]):
        """
        parameters:
            outcomes:
                all possible outcomes of the state, TICKING and ABORT will be added.
        """
        self.outcomes = outcomes
        self.outcomes.append(TICKING)
        self.outcomes.append(ABORT)
        super().__init__(self.outcomes)
        self.status = TickingState_Status.ENTRY
        self.outcome = "" # will contain the last used outcome
        #self.log = DummyLogger #YasminNode.get_instance().get_logger()
    
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
            #self.log.info("TickintState.execute ENTRY")           
            try:
                self.outcome = self.entry(blackboard)
                #self.log.info(f"TickintState.execute ENTRY returns {self.outcome}")
            except Exception as e:
                print("exception occurred : "+ traceback.format_exc())
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
                self.log.info("exception occured : "+ traceback.format_exc())
                self.outcome = ABORT
                self.status = TickingState_Status.EXIT
            if self.outcome == TICKING:
                return TICKING
            self.status = TickingState_Status.EXIT

        if self.status == TickingState_Status.EXIT:
            self.outcome = self.exit()
            self.status = TickingState_Status.ENTRY
            return self.outcome
        self.log.info("at end of execute")
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
        return TickingState_Status.DOO;

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




class cbStateMachine(TickingState):
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
    def __init__(self, outcomes: List[str], transitioncb=default_transitioncb, statecb=default_statecb) -> None:
        outcomes.append(TICKING)
        super().__init__(outcomes)

        self._states = {}
        self._start_state = None
        self.__current_state = None
        self.__current_state_lock = Lock()
        self.statecb = statecb
        self.transitioncb = transitioncb

    def add_state(
        self,
        name: str,
        state: State,
        transitions: Dict[str, str] = None
    ) -> None:
        if not transitions:
            transitions = {}
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

    def execute(self, blackboard: Blackboard) -> str:

        #with self.__current_state_lock:
        #    self.__current_state = self._start_state
        while True:
            with self.__current_state_lock:
                state = self._states[self.__current_state]
                name = self.__current_state
            self.statecb(self,blackboard,name)
            outcome = state["state"](blackboard)

            # check outcome belongs to state
            if outcome not in state["state"].get_outcomes():
                with self.__current_state_lock:
                    self.__current_state = self._start_state
                raise Exception(
                    f"Outcome ({outcome}) is not register in state {self.__current_state}")


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
                raise Exception(f"Outcome ({outcome}) without transition")

    def get_states(self) -> Dict[str, Union[State, Dict[str, str]]]:
        return self._states

    def get_current_state(self) -> str:
        with self.__current_state_lock:
            if self.__current_state:
                return self.__current_state

        return ""

    def __str__(self) -> str:
        return f"StateMachine: {self._states}"
    


#
#
# End copied part from yasmin_action
#


# def nested_etasl_state(name: str, file_path: str, robot_path: str, display_in_viewer: bool= False):
# class eTaSL_StateMachine(cbStateMachine):
#     """
#     A sub statemachine that:
#     - uses cbStateMachine to provide callbaxks for transtions and state changes
#     - uses a feedback to set parameters
#     - scopes the names of the state, such that the feedback trace is understandable.
#     - separate name of the state from the name of the task
#     """
#     def __init__(self,name: str,  
#                  task: str = None,
#                 display_in_viewer: bool= False, 
#                 setparamcb = default_parameter_setter,
#                 transitioncb=default_transitioncb, 
#                 statecb=default_statecb):
#         super().__init__(outcomes=[SUCCEED, ABORT],transitioncb=transitioncb,statecb=statecb)

#         if task is None:
#             task = name

#         self.add_state(name+".DEACTIVATE_ETASL", DeactivateEtasl(),
#                 transitions={SUCCEED: name+".CLEANUP_ETASL",
#                             ABORT: name+".CLEANUP_ETASL",
#                             TIMEOUT: ABORT}) #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues

#         self.add_state(name+".CLEANUP_ETASL", CleanupEtasl(),
#                 transitions={SUCCEED: name+".PARAMETER_CONFIG",
#                             ABORT: name+".PARAMETER_CONFIG",
#                             TIMEOUT: ABORT}) #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues

#         self.add_state(name+".PARAMETER_CONFIG", ReadTaskParametersCB(task,setparamcb),
#                 transitions={SUCCEED: name+".ROBOT_SPECIFICATION",
#                             ABORT: ABORT,
#                             TIMEOUT: ABORT})

#         self.add_state(name+".ROBOT_SPECIFICATION", ReadRobotSpecificationFile(task),
#                 transitions={SUCCEED: name+".TASK_SPECIFICATION",
#                             ABORT: ABORT,
#                             TIMEOUT: ABORT})

#         self.add_state(name+".TASK_SPECIFICATION", ReadTaskSpecificationFile(name),
#                 transitions={SUCCEED: name+".CONFIG_ETASL",
#                             ABORT: ABORT,
#                             TIMEOUT: ABORT})

#         self.add_state(name+".CONFIG_ETASL", ConfigureEtasl(),
#                 transitions={SUCCEED: name+".ACTIVATE_ETASL",
#                             ABORT: ABORT,
#                             TIMEOUT: ABORT})

#         #state_name = f"RUNNING_{name}" 

#         self.add_state(name+".ACTIVATE_ETASL", ActivateEtasl(),
#                 transitions={SUCCEED: name+".RUNNING",
#                             ABORT: ABORT,
#                             TIMEOUT: ABORT})

#         self.add_state(name+".RUNNING", Executing(name),
#                 transitions={"e_finished@etasl_node": SUCCEED,
#                             ABORT: ABORT})

#         if display_in_viewer:
#             YasminViewerPub('{} (nested FSM)'.format(name), self)

  

# ## some experimentation:

# class TestState(State):
#     def __init__(self) -> None:
#         super().__init__([SUCCEED,"iterate"])
#         self.count = 0

#     def execute(self, blackboard: Blackboard) -> str:

#         return SUCCEED
    


# def monitor( blackboard:Blackboard):
#     # initialisation
#     for i in range(10):
#         # do work
#         yield "continue"
#     # cleanup
#     yield "success"



def default_coroutine(self, blackboard:Blackboard):
    yield SUCCEED

# class DummyLogger:
#     def info(s:str):
#         pass

class Generator(TickingState):
    """
    Uses a python generator to define a TickingState. It implements methods `entry`, `doo`, and `exit` of TickingState using
    the given callback function (which is a python generator that can return intermediate results usig `yield`).  This makes
    it easy to specify a TickingState.

    The callback that needs to be defined yields strings representing outcomes:
    - if TICKING is yielded, expects to be called again, otherwise expects that this is
      the end of the task.
    """
    def __init__(self, name,outcomes: List[str], execute_cb= default_coroutine) -> None:
        """
        parameters:
            name:
                name of the node
            outcomes:
                a list of strings indicating the expected outcomes,  TICKING and ABORT will be
                automatically added.
            executecb:
                callback function, is a python generator function (i.e. containing `yield`)
        """
        super().__init__(outcomes)
        self.name = name
        self.co_execute = execute_cb

    def cancel_state(self) -> None:
        super().cancel_state()

    def entry(self, blackboard: Blackboard) -> str:
        self.generator = self.co_execute(self,blackboard)
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
        return self.outcome
    
    def reset(self):
        super().reset()

    def __str__(self) -> str:
        pass



# class Sequence(TickingState):
#     """
#     Implements a behaviortree-like Sequence
#     SUCCESS=SUCCEED outcome,
#     FAILURE=any other outcome

#     minimizes the number of ticks.
#     """
#     def __init__(self, name,outcomes: List[str], statecb=default_statecb) -> None:
#         super().__init__(outcomes)
#         self.states=[]
#         self.statecb = statecb  # NOT IMPLEMENTED, NEED TO REDEFINE statecb SIGNATURE (not depend on StateMachine)
#         self.count = 0
#         self.log = YasminNode.get_instance().get_logger()
#         self.name = name

#     def add_state(self, name:str, state: State) -> None:
#         self.states.append({"name":name,"state":state})        

#     def cancel_state(self) -> None:
#         super().cancel_state()

#     def entry(self, blackboard: Blackboard) -> str:
#         self.log.info(f"{self.name} : start sequence")
#         self.count=0
#         if len(self.states)==0:
#             return SUCCEED
#         return CONTINUE

#     def doo(self,blackboard:Blackboard) -> str:
#         state = self.states[self.count]["state"]
#         while True:
#             #self.log.info(f"{self.name} : sequence {self.count}")
#             outcome=state(blackboard)
#             if outcome==TICKING:
#                 return outcome
#             elif outcome==SUCCEED:
#                 self.count = self.count+1                
#                 if self.count >= len(self.states):
#                     self.outcome=SUCCEED
#                     return SUCCEED
#                 state=self.states[self.count]["state"]                
#             else:
#                 self.outcome=outcome
#                 return outcome
        

#     def exit(self) -> str:
#         self.log.info(f"{self.name} : sequence finished with outcome {self.outcome}")
#         return self.outcome
    
#     def reset(self):
#         for s in self.states:
#             if isinstance(s["state"],TickingState):
#                 print("reset state : " + s["name"])
#                 s["state"].reset()
#         super().reset()


#     def __str__(self) -> str:
#         pass


class Sequence(Generator):
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
    def __init__(self, name:str,outcomes: List[str]) -> None:
        """
        parameters:
            name: 
                name of the sequence
            outcomes: 
                all the allowable outcomes of the Sequence. TICKING and ABORT will be added
        """
        super().__init__(name,outcomes,execute_cb=Sequence.co_execute)
        self.states=[]
        self.count = 0
        self.log = YasminNode.get_instance().get_logger()

    def add_state(self, name:str, state: State):
        """
        adds a state to the sequence
        parameters:
            name:
                name of the state
            state:
                state intance
        returns:
            Sequence object (to allow method chaining)
        """
        self.states.append({"name":name,"state":state})  
        return self 
    
    def reset(self):  # general rule, if you own states, you have to reset them
        """
        resets the sequence and ensures tht the underlying states are also reset.
        """
        for s in self.states:
            if isinstance(s["state"],TickingState):
                print("reset state : " + s["name"])
                s["state"].reset()
        super().reset()    

    def co_execute(self,blackboard):
        for s in self.states:                
            outcome = s["state"](blackboard)
            while outcome==TICKING:
                yield TICKING
                outcome = s["state"](blackboard)
            if outcome!=SUCCEED:
                yield outcome                            
        yield SUCCEED


class ConcurrentSequence(Generator):
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
    def __init__(self, name:str,outcomes: List[str]) -> None:
        """
        parameters:
            name: 
                name of the sequence
            outcomes: 
                all the allowable outcomes of the Sequence. TICKING and ABORT will be added
        """
        super().__init__(name,outcomes,execute_cb=ConcurrentSequence.co_execute)
        self.states=[]
        self.count = 0
        #self.log = YasminNode.get_instance().get_logger()

    def add_state(self, name:str, state: State):
        """
        adds a state to the sequence
        parameters:
            name:
                name of the state
            state:
                state intance
        returns:
            Sequence object (to allow method chaining)
        """
        self.states.append({"active": True,"name":name,"state":state})  
        return self 
    
    def reset(self): 
        """
        resets the sequence and ensures tht the underlying states are also reset.
        """
        for s in self.states:
            if isinstance(s["state"],TickingState):
                s["state"].reset()
        super().reset()    

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
        



class Fallback(Generator):
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
    def __init__(self, name:str, outcomes, statecb=default_statecb) -> None:
        """
        parameters:
            name: 
                name of the sequence
            outcomes: 
                all the allowable outcomes of the Sequence. TICKING and ABORT will be added
        """        
        super().__init__(name,outcomes,execute_cb=Fallback.co_execute,statecb=statecb)
        self.states=[]
        self.count = 0
        self.log = YasminNode.get_instance().get_logger()
        
    def add_state(self, name:str, state: State):
        """
        adds a state to the fallback node
        parameters:
            name:
                name of the state
            state:
                state intance
        returns:
            Sequence object (to allow method chaining)
        """        
        self.states.append({"name":name,"state":state})  
        return self 
    
    def reset(self): 
        """
        resets the sequence and ensures tht the underlying states are also reset.
        """        
        for s in self.states:
            if isinstance(s["state"],TickingState):
                print("reset state : " + s["name"])
                s["state"].reset()
        super().reset()    

    def co_execute(self,blackboard):
        for s in self.states:                
            outcome = s["state"](blackboard)
            while outcome==TICKING:
                yield TICKING
                outcome = s["state"](blackboard)
            if outcome!=CANCEL:
                yield outcome                            
        yield CANCEL



class ConcurrentFallback(Generator):
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
    def __init__(self, name:str,outcomes: List[str]) -> None:
        """
        parameters:
            name: 
                name of the sequence
            outcomes: 
                all the allowable outcomes of the Sequence. TICKING and ABORT will be added
        """
        super().__init__(name,outcomes,execute_cb=ConcurrentSequence.co_execute)
        self.states=[]
        self.count = 0
        self.log = YasminNode.get_instance().get_logger()

    def add_state(self, name:str, state: State):
        """
        adds a state to the sequence
        parameters:
            name:
                name of the state
            state:
                state intance
        returns:
            Sequence object (to allow method chaining)
        """
        self.states.append({"active": True,"name":name,"state":state})  
        return self 
    
    def reset(self): 
        """
        resets the sequence and ensures tht the underlying states are also reset.
        """
        for s in self.states:
            if isinstance(s["state"],TickingState):
                s["state"].reset()
        super().reset()    

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
                callback function with signature `condition(self, blackboard:Blackboard) -> bool`

        """
        outcomes = ["SUCCEED"]
        super().__init__("WaitFor",outcomes, execute_cb = WaitFor.co_execute)
        self.condition_cb = condition_cb

    def co_execute(self,blackboard):
        while not self.condition_cb(self,blackboard):
            yield TICKING
        yield SUCCEED

    def reset(self):  # general rule, if you own states, you have to reset them
        if self.state is not None and isinstance(self.state,TickingState):
            self.state.reset()
        super().reset()    



class ConditionWhile(Generator):
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
    def __init__(self, condition_cb:Callable, state:TickingState):
        """
        Same behavior as the underlying state `state`, but returns CANCEL if condition_cb(self,condition_cb)
        ever gets false. 


        Parameters:
            condition_cb:
                callback function with signature `condition(self, blackboard:Blackboard) -> bool`
            state:
                at each tick of the underlying state, the condition_cb is checked 
        """
        outcomes = state.get_outcomes()
        outcomes.append(CANCEL)  # Generator will add TICKING and ABORT
        super().__init__("ConditionWhile",outcomes, execute_cb = ConditionWhile.co_execute)
        self.condition_cb = condition_cb
        self.state=state
    
    def co_execute(self,blackboard):        
        while self.condition_cb(self,blackboard):
            outcome = self.state(blackboard)
            yield outcome
        yield CANCEL

    def reset(self):  # general rule, if you own states, you have to reset them
        if self.state is not None and isinstance(self.state,TickingState):
            self.state.reset()
        super().reset()    


class Repeat(Generator):
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
    def __init__(self,maxcount:int, state: TickingState):
        """Repeats underlying state for *maxcount* times or until an outcome other than SUCCEED is reached

        Parameters:
            maxcount: 
                Maximum number of times that underlying state will be executed.
            state:
                underlying state
        """         
        outcomes = state.get_outcomes()
        super().__init__("Repeat",outcomes,execute_cb=Repeat.co_execute)
        self.state = state
        self.maxcount = maxcount
        self.log = YasminNode.get_instance().get_logger()
    def co_execute(self,blackboard):        
        for c in range(self.maxcount):
            outcome = self.state(blackboard)
            while outcome==TICKING:
                yield TICKING
                outcome = self.state(blackboard)
            if outcome!=SUCCEED:
                yield outcome
        yield SUCCEED
    def reset(self):  # general rule, if you own states, you have to reset them
        if isinstance(self.state,TickingState):
            self.state.reset()
        super().reset()    




# class JoinStateMachines(TickingState):
#     """
#     executes state machines in parallel and waits for both of them to exit ("Join")
#     Synchronous execution, starting in the order of adding
#     The outcome TICKING will be executed in an interleaved way.
#     only has a statecb, because it does not perform any transitions (underlying statemachine however do)
#     still checks for outcomes to conform spec.


#     outcome of the join:
#         wait until all state machines return SUCCEED and return SUCCEED
#         TICKING outcome of underlying statemachines propagates the tick as outcome of this class
#         any other outcome that first arrives cancels the others and returns this outcome
#           (e.g. failure)
#     """    
#     def __init__(self, outcomes: List[str], statecb=default_statecb) -> None:
#         #outcomes.append(TICKING)
#         super().__init__(outcomes)

#         self.states = []
#         self.statecb = statecb
#         self.count=0
#         self.countActive=0
#         self.log = YasminNode.get_instance().get_logger()

#     def add_state(
#         self,
#         name: str,
#         state: State
#     ):
#         self.states.append( {"name":name, "state":state,'active':True})
#         return self

#     def cancel_state(self) -> None:
#         for s in self.states: 
#             s["state"].cancel_state()
#         super().cancel_state()


#     def entry(self, blackboard: Blackboard) -> str:
#         #self.log.info(f"JoinStateMachines: entry called")
#         if len(self.states)==0:
#             raise RuntimeError("At least one state should be added to JoinStateMachines")
#         self.count=0
#         for s in self.states:
#             s["active"] = True
#         #self.countActive = len(self.states)
#         return CONTINUE;

#     def doo_old(self, blackboard: Blackboard) -> str:
#         #self.log.info(f"JoinStateMachines: doo() called ({len(self.states)=},{self.count=},{self.countActive=})")
#         while self.countActive>0:
#             if self.states[self.count]["active"]:
#                 self.outcome = self.states[self.count]["state"](blackboard)
#                 if self.outcome==TICKING:
#                     self.count = (self.count + 1) % len(self.states)
#                     return self.outcome
#                 elif self.outcome==SUCCEED:
#                     self.states[self.count]["active"]=False
#                     self.countActive = self.countActive - 1
#                 else:
#                     # other outcome than SUCCEED, return (exit will reset the rest of the states)
#                     self.states[self.count]["active"]=False
#                     self.countActive = self.countActive - 1
#                     return self.outcome                    
#             self.count = (self.count + 1) % len(self.states)
#         return SUCCEED

#     def doo(self,blackboard:Blackboard) -> str:
#         """
#         execute in parallel while avoiding **all** unnecessary ticks:
#         - go through all the states:
#             if active==True, execute and if outcome== :
#             - TICKING : active=True,
#             - SUCCEED or other : active=False,
#         - if any other outcome, return first other outcome (the other parallel states are still executed once)
#         - if any TICKING, return TICKING
#         - otherwise (if all SUCCEED): return SUCCEED
#         """
#         ticking=0        
#         other=0
#         for s in self.states:
#             if s["active"]:
#                 outcome = s["state"](blackboard)
#                 if outcome==TICKING:    # ticking
#                     ticking=ticking+1
#                 elif outcome!=SUCCEED:  # other outcome:
#                     if other==0:
#                         self.outcome=outcome
#                     s["active"] = False
#                     other=other+1
#                 else:                   # SUCCEED
#                     s["active"] = False
#         # if any other outcome, we consider this a failure and return this outcome 
#         #    (even if somebody else is TICKING)
#         #    (self.outcome will the outcome of first state failing )
#         #    (TickingState.execute() will exit)
#         if other!=0 :
#             return self.outcome
#         # if any is ticking, return TICKING 
#         #    (TickingState.execute() will call us back to continue in the next tick)
#         if ticking != 0:
#             self.outcome=TICKING
#             return TICKING
#         # otherwise, all was succesfull and we can return SUCCEED
#         #    (TickingState.execute() will exit)
#         self.outcome = SUCCEED
#         return SUCCEED

#     def exit(self) -> str:
#         self.log.info(f"JoinStateMachines: exit called")
#         # returns by default the outcome of entry or doo method.
#         for s in self.states:
#             s["state"].reset()        
#         return self.outcome
    
#     def reset(self) -> str:
#         for s in self.states:
#             if isinstance(s["state"],TickingState):
#                 s["state"].reset()                
#         super().reset()

#     def __str__(self) -> str:
#         return f"JoinStateMachine: {self.states}"
    