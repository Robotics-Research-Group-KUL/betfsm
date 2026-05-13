
class JoinStateMachines(TickingState):
    """
    executes state machines in parallel and waits for both of them to exit ("Join")
    Synchronous execution, starting in the order of adding
    The outcome TICKING will be executed in an interleaved way.
    only has a statecb, because it does not perform any transitions (underlying statemachine however do)
    still checks for outcomes to conform spec.


    outcome of the join:
        wait until all state machines return SUCCEED and return SUCCEED
        TICKING outcome of underlying statemachines propagates the tick as outcome of this class
        any other outcome that first arrives cancels the others and returns this outcome
          (e.g. failure)
    """    
    def __init__(self, outcomes: List[str], statecb=default_statecb) -> None:
        #outcomes.append(TICKING)
        super().__init__(outcomes)

        self.states = []
        self.statecb = statecb
        self.count=0
        self.countActive=0
        self.log = YasminNode.get_instance().get_logger()

    def add_state(
        self,
        name: str,
        state: State
    ):
        self.states.append( {"name":name, "state":state,'active':True})
        return self

    def cancel_state(self) -> None:
        for s in self.states: 
            s["state"].cancel_state()
        super().cancel_state()


    def entry(self, blackboard: Blackboard) -> str:
        #self.log.info(f"JoinStateMachines: entry called")
        if len(self.states)==0:
            raise RuntimeError("At least one state should be added to JoinStateMachines")
        self.count=0
        for s in self.states:
            s["active"] = True
        #self.countActive = len(self.states)
        return CONTINUE;

    def doo_old(self, blackboard: Blackboard) -> str:
        #self.log.info(f"JoinStateMachines: doo() called ({len(self.states)=},{self.count=},{self.countActive=})")
        while self.countActive>0:
            if self.states[self.count]["active"]:
                self.outcome = self.states[self.count]["state"](blackboard)
                if self.outcome==TICKING:
                    self.count = (self.count + 1) % len(self.states)
                    return self.outcome
                elif self.outcome==SUCCEED:
                    self.states[self.count]["active"]=False
                    self.countActive = self.countActive - 1
                else:
                    # other outcome than SUCCEED, return (exit will reset the rest of the states)
                    self.states[self.count]["active"]=False
                    self.countActive = self.countActive - 1
                    return self.outcome                    
            self.count = (self.count + 1) % len(self.states)
        return SUCCEED

    def doo(self,blackboard:Blackboard) -> str:
        """
        execute in parallel while avoiding **all** unnecessary ticks:
        - go through all the states:
            if active==True, execute and if outcome== :
            - TICKING : active=True,
            - SUCCEED or other : active=False,
        - if any other outcome, return first other outcome (the other parallel states are still executed once)
        - if any TICKING, return TICKING
        - otherwise (if all SUCCEED): return SUCCEED
        """
        ticking=0        
        other=0
        for s in self.states:
            if s["active"]:
                outcome = s["state"](blackboard)
                if outcome==TICKING:    # ticking
                    ticking=ticking+1
                elif outcome!=SUCCEED:  # other outcome:
                    if other==0:
                        self.outcome=outcome
                    s["active"] = False
                    other=other+1
                else:                   # SUCCEED
                    s["active"] = False
        # if any other outcome, we consider this a failure and return this outcome 
        #    (even if somebody else is TICKING)
        #    (self.outcome will the outcome of first state failing )
        #    (TickingState.execute() will exit)
        if other!=0 :
            return self.outcome
        # if any is ticking, return TICKING 
        #    (TickingState.execute() will call us back to continue in the next tick)
        if ticking != 0:
            self.outcome=TICKING
            return TICKING
        # otherwise, all was succesfull and we can return SUCCEED
        #    (TickingState.execute() will exit)
        self.outcome = SUCCEED
        return SUCCEED

    def exit(self) -> str:
        self.log.info(f"JoinStateMachines: exit called")
        # returns by default the outcome of entry or doo method.
        for s in self.states:
            s["state"].reset()        
        return self.outcome
    
    def reset(self) -> str:
        for s in self.states:
            if isinstance(s["state"],TickingState):
                s["state"].reset()                
        super().reset()

    def __str__(self) -> str:
        return f"JoinStateMachine: {self.states}"
    



class Sequence(TickingState):
    """
    Implements a behaviortree-like Sequence
    SUCCESS=SUCCEED outcome,
    FAILURE=any other outcome

    minimizes the number of ticks.
    """
    def __init__(self, name,outcomes: List[str], statecb=default_statecb) -> None:
        super().__init__(outcomes)
        self.states=[]
        self.statecb = statecb  # NOT IMPLEMENTED, NEED TO REDEFINE statecb SIGNATURE (not depend on StateMachine)
        self.count = 0
        self.log = YasminNode.get_instance().get_logger()
        self.name = name

    def add_state(self, name:str, state: State) -> None:
        self.states.append({"name":name,"state":state})        

    def cancel_state(self) -> None:
        super().cancel_state()

    def entry(self, blackboard: Blackboard) -> str:
        self.log.info(f"{self.name} : start sequence")
        self.count=0
        if len(self.states)==0:
            return SUCCEED
        return CONTINUE

    def doo(self,blackboard:Blackboard) -> str:
        state = self.states[self.count]["state"]
        while True:
            #self.log.info(f"{self.name} : sequence {self.count}")
            outcome=state(blackboard)
            if outcome==TICKING:
                return outcome
            elif outcome==SUCCEED:
                self.count = self.count+1                
                if self.count >= len(self.states):
                    self.outcome=SUCCEED
                    return SUCCEED
                state=self.states[self.count]["state"]                
            else:
                self.outcome=outcome
                return outcome
        

    def exit(self) -> str:
        self.log.info(f"{self.name} : sequence finished with outcome {self.outcome}")
        return self.outcome
    
    def reset(self):
        for s in self.states:
            if isinstance(s["state"],TickingState):
                print("reset state : " + s["name"])
                s["state"].reset()
        super().reset()


    def __str__(self) -> str:
        pass


#def nested_etasl_state(name: str, file_path: str, robot_path: str, display_in_viewer: bool= False):
class eTaSL_StateMachine(cbStateMachine):
    """
    A sub statemachine that:
    - uses cbStateMachine to provide callbaxks for transtions and state changes
    - uses a feedback to set parameters
    - scopes the names of the state, such that the feedback trace is understandable.
    - separate name of the state from the name of the task
    """
    def __init__(self,name: str,  
                 task: str = None,
                display_in_viewer: bool= False, 
                setparamcb = default_parameter_setter,
                transitioncb=default_transitioncb, 
                statecb=default_statecb):
        super().__init__(outcomes=[SUCCEED, ABORT],transitioncb=transitioncb,statecb=statecb)

        if task is None:
            task = name

        self.add_state(name+".DEACTIVATE_ETASL", DeactivateEtasl(),
                transitions={SUCCEED: name+".CLEANUP_ETASL",
                            ABORT: name+".CLEANUP_ETASL",
                            TIMEOUT: ABORT}) #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues

        self.add_state(name+".CLEANUP_ETASL", CleanupEtasl(),
                transitions={SUCCEED: name+".PARAMETER_CONFIG",
                            ABORT: name+".PARAMETER_CONFIG",
                            TIMEOUT: ABORT}) #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues

        self.add_state(name+".PARAMETER_CONFIG", ReadTaskParametersCB(task,setparamcb),
                transitions={SUCCEED: name+".ROBOT_SPECIFICATION",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        self.add_state(name+".ROBOT_SPECIFICATION", ReadRobotSpecificationFile(task),
                transitions={SUCCEED: name+".TASK_SPECIFICATION",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        self.add_state(name+".TASK_SPECIFICATION", ReadTaskSpecificationFile(name),
                transitions={SUCCEED: name+".CONFIG_ETASL",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        self.add_state(name+".CONFIG_ETASL", ConfigureEtasl(),
                transitions={SUCCEED: name+".ACTIVATE_ETASL",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        #state_name = f"RUNNING_{name}" 

        self.add_state(name+".ACTIVATE_ETASL", ActivateEtasl(),
                transitions={SUCCEED: name+".RUNNING",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        self.add_state(name+".RUNNING", Executing(name),
                transitions={"e_finished@etasl_node": SUCCEED,
                            ABORT: ABORT})

        if display_in_viewer:
            YasminViewerPub('{} (nested FSM)'.format(name), self)

  

## some experimentation:

class TestState(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED,"iterate"])
        self.count = 0

    def execute(self, blackboard: Blackboard) -> str:

        return SUCCEED
    


def monitor( blackboard:Blackboard):
    # initialisation
    for i in range(10):
        # do work
        yield "continue"
    # cleanup
    yield "success"



class DummyLogger:
    def info(s:str):
        pass

#
# blackboard.task[taskname].parameters = dict with task parameters
#

#
# For now, this  is a copy of cb_state_machine.py from yasmin_action.
#
#
def default_parameter_setter(blackboard:Blackboard, parameters:dict) -> dict:
    """
    default do-nothing callback function to set parameters.

    Input
    -----
    - blackboard: the blackboard, in case additional information is needed to construct the parameters
    - parameters: parameters as read from tasks-schema.json

    Output:
    -------
    - updated parameters
    """
    return parameters

class ReadTaskParametersCB(ServiceState):
    """Instructs the eTaSL node to read in parameters for the given task with a callback function
    
       could replace the original ReadTaskParameters without much compatibility issues.
    """
    def __init__(self, task_name: str, setparamcb: default_parameter_setter) -> None:
        super().__init__(
            TaskSpecificationString,  # srv type
            "/etasl_node/readTaskSpecificationString",  # service name
            self.create_request_handler,  # cb to create the request
            [],  # outcomes. Includes SUCCEED, ABORT, TIMEOUT by default
            self.response_handler,  # cb to process the response
            timeout = 2.5 #seconds 
        )
        self.task_name = task_name
        self.setparamcb = setparamcb
        # self.file_path = file_path

    def create_request_handler(self, blackboard: Blackboard) -> TaskSpecificationString.Request:

        req = TaskSpecificationString.Request()

        task = get_task(self.task_name,blackboard)

        # filter and copy task["parameters"]
        param = {}
        for key,value in task["parameters"].items():
            if (key[:3]!="is-") and (key!="file_path"):
                param[key] = value
        paramdef = param.copy()
        param = self.setparamcb(blackboard, param)
        param_string = ""
        # parameter checking using paramdef
        for key,value in param.items():
            if key not in paramdef:
                raise ValueError(f"callback sets {key} parameter that is not in schema ")
            if value == "external":
                raise ValueError(f"parameter declared 'external' is not set by callback")
        for key,_ in paramdef.items():
            if key not in param:
                raise ValueError(f"required parameter {key} is not defined" )
        
        # lua script setting parameters:
        for key, value in param.items():
            if isinstance(value,bool):
                param_string = param_string + f"{key} = {str(value).lower()}\n"
            else:
                param_string = param_string + f"{value}"

        req.str = param_string

        print(f"{param_string=}")
        # print("ReadTaskSpecificationString")
        return req

    def response_handler(self,blackboard: Blackboard,response: TaskSpecificationString.Response) -> str:

        # print("Service response success: " + str(response.success))
        blackboard.success = response.success
        if not response.success:
            return ABORT
        # time.sleep(1)
        return SUCCEED



class TopicState(Generator):
    """
    timeout using a composition with a Timeout() class, not implemented here.

    - messages queued and list passed to callback `cb` (that will determine policy of dealing with multiple messages)
    - `cb` can change blackboard
    - `cb` can determine outcome
    - after `cb` the queue is cleared.
    - TICKING if no topics received, outcome returned by `cb` otherwise.34


    Todo:
        - finish implementation
        - clears queue of messages before state is started. ?? (risk of losing messages
          when used in a statemachine that loops and reacts to outcomes ?) ;
            an entry_cb,doo_cb ? where entry_cb receives all previous messages.
        - do we need an underlying state/statemachine
        - when CALL outcome, call underlying state?
        - a bool variable to only store when state is active (controlled from cb's ? entry, doo, exit ?)
        - A topic state that contains an underlying state, together with a queuing mechanism and a state that the underlying state can
          use to transition.  Decoupling the receiving messages scope from the point(s) of generating transitions.

    2nd generation of design:
        - A state machine that additionally listens and can inject additional transitions. (closest to rFSM)
        - ? This state machine before returninG TICKING, listens to a queue and checks with a policy the transitions. The states outcomes have priority
        - ? the qeuue is a priority queue where each record has an outcome and a priority.  lowest number first, state itself is zero. positive only when ticking
          negative will push outcome of state and  interrupting with higher priority. (better of abort scenario's)
        - queue:
            - has a name and stored in blackboard, can be used with multiple statemachines.
            - has a list of allowable outcomes
            - an element of the queue has an outcome, priority and payload
        - Queue state:
            - defines queue
            - registers multiple listeners

    3th generation of design:
        - Priority,  
            - a non-ticking outcome of a state has priority zero, 
            - TICKING always yield to the queue (lowest priority possible)
            - highest priority number has the priority.
            - by default outcomes put in the queue externally will have priority -10,
        - specialisation of cbStateMachine:
            - that additionally registers Listeners and calls them just after the underlying states return an outcome
            - are "shallow": only deal with outcomes of the state machine, not inside the underlying states/state machine.
            - many other classes needs such a listener input during construction
        - Each listener:
            - contains a queue (from queue import PriorityQueue, customers.put((2, "Harry")), customers.get()   )
                - with outcome
                - with priority
                - the priorityqueue can deal with concurrency.
            - can be used with multiple state machines.
            - can be chained together
            - if an outcome is used, it is consumed. only be used once!
            - if it only wants to adapt the blackboard, it leaves the queue empty.
            - it has a callback:
                - to transform the received topic messages to the queue.
                - to write payload to blackboard.
                - processes all topic messages received after last call of callback.
    """
    def __init__(
            self, 
            topic_name:str, 
            topic_type: Type,
            outcomes: List[str],
            cb: Callable,            
            queue_size: int = 30,
            state : TickingState = None,
            node: Node = None
            ):
        """
        Parameters:
            topic_name:
                name of the topic
            topic_type:
                type of the topic
            outcomes:
                allowable outcomes.
            cb:
                a callback function with signature `def cb(self,blackboard, msg_queue)`, will be synchronously called
                at each call of the state (i.e. while ticking). msg_queue can contain multiple messages.
                the callback function returns an outcome that will be yielded.                
            queue_size:
                max. queue size for the msg_queue passed in the callback (and indicated to middleware)
            state:
                underlying state, can be None if there is no underlying state.
            node:
                ROS2 node, by default YasminNode.get_instance()
        """
        super().__init__("TopicState",outcomes, TopicState.co_execute)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, #Keeps the last msgs received in case buffer is fulll
            depth=queue_size, #Buffer size
            reliability=QoSReliabilityPolicy.RELIABLE, #Uses TCP for reliability instead of UDP
            durability=QoSDurabilityPolicy.VOLATILE #Volatile, may not use first msgs if subscribed late (will not happen in this context)
        )
        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = node
        self.monitoring=False
        self.subscription = self.node.create_subscription(
            topic_type, topic_name, self.topic_callback, qos_profile)

    def callback_msg(self, msg) -> None:

        if self.monitoring:
            self.msg_list.append(msg)

            if len(self.msg_list) >= self.msg_queue:
                self.msg_list.pop(0)


    def take_msgs(self) -> List:
        # protect with lock?
        msgs = self.msg_list.copy()
        self.msg_list.clear();
        return msgs


    def topic_co_execute(self,blackboard, msgs):
        pass


    def co_execute(self,blackboard):
        pass
