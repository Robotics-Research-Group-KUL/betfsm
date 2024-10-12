"""
Yasmin_Ticking states related to eTaSL
"""

from yasmin_ticking_ros import *

from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString


import json
from jsonschema import validate, exceptions




def get_task(blackboard:Blackboard,task_name:str) -> List[str|List]:
    """
    get the default parameters for a task.

    Parameters:
        blackboard: blackboard
        task_name: name of the task
    """
    try:
        tasks=blackboard["tasks"]
    except:
        raise Exception("There is no `tasks` defined in the blackboard, are you sure you loaded a list of tasks?")
    for task in tasks:
        if task.get("name")==task_name:
            if not("parameters" in task):
                raise Exception("task dictionary should have `parameters` keyword")
            return task
    raise Exception(f"No task with name {task_name} found in the blackboard")





def load_task_list( json_file_name: str, blackboard: Blackboard) -> None:
    """
    Loads a task list from a file. References to packages and environment variables in the name
    are expanded (using the expand_... functions)

    Parameters:
        json_file_name: 
            json file containing the task list.
        blackboard:
            blackboard into which to load the task list.
    """
    with open(expand_ref(json_file_name), 'r') as json_file:
        parameters = json.load(json_file)
        blackboard["tasks"] = parameters["tasks"]


def default_parameter_setter(self,param):
    return param

class SetTaskParameters(ServiceClient):
    def __init__(self,
                 task_name:str,
                 srv_name:str = "/etasl_node",
                 cb: Callable = default_parameter_setter,
                 timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None):
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(srv_name+"/readTaskSpecificationString",TaskSpecificationString,outcomes,timeout,node)
        self.task_name = task_name
        self.cb = cb

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        task = get_task(blackboard,self.task_name)
        # eliminate irrelevant parameters and put in a local dictionary:
        # (parameter changes should be local, not global)
        param = {}
        for key,value in task["parameters"].items():
            if (key[:3]!="is-") and (key!="file_path"):
                param[key] = value
        param_definition = param.copy()
        # calling callback
        param = self.cb(self,param)
        # parameter checking using paramdef
        for key,value in param.items():
            if key not in param_definition:
                raise ValueError(f"callback sets {key} parameter that is not in schema ")
            # not really needed anymore:
            if value == "external":
                raise ValueError(f"parameter declared 'external' is not set by callback")          
        # constructing LUA script fragment:       
        param_string = ""
        for key, value in param.items():
            if isinstance(value,bool):
                param_string = param_string + f"{key} = {str(value).lower()}\n"
            else:
                param_string = param_string + f"{key} = {value}\n"
        # constructing request:
        request.str  = param_string
        self.log.info(f"Set parameters for eTaSL task {self.task_name}\n{param_string}")
        return request    
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            return ABORT




class SetRobotSpecification(ServiceClient):
    def __init__(self,
                 task_name:str,
                 srv_name:str = "/etasl_node",
                 timeout:Duration = Duration(seconds=1.0), 
                 node:Node = None):
        outcomes=[SUCCEED]  # ABORT + TIMEOUT added
        super().__init__(
            srv_name+"/readTaskSpecificationFile",
            TaskSpecificationString, outcomes, timeout,node)
        self.task_name = task_name

    def fill_in_request(self, blackboard: Blackboard, request) -> None:
        # lookup task.
        for task in blackboard["tasks"]:
            if task["name"]==self.task_name:
                specfile = task.get["robot_specification_file"]
                if specfile is None or specfile=="":
                    specfile = blackboard["default_robot_specification"]
                    if specfile is None or specfile=="":
                        raise Exception("No robot_specification_file defined")
                request.str  = specfile
                print("robot specification file ",specfile)
                return request
            
        raise Exception(f"Task with name {self.task_name} was not found")
    
    def process_result(self, blackboard: Blackboard, response) -> str:
        if response.success:
            return SUCCEED
        else: 
            return ABORT





 
# def nested_etasl_state(name: str, file_path: str, robot_path: str, display_in_viewer: bool= False):
class eTaSL_StateMachine(cbStateMachine):
    """
    A sub statemachine that:
    - uses cbStateMachine to provide callbaxks for transtions and state changes
    - uses a feedback to set parameters
    - scopes the names of the state, such that the feedback trace is understandable.
    - separate name of the state from the name of the task
    """
    def __init__(self,
                 task_name: str = None,
                 srv_name: str = "/etasl_node",
                 #display_in_viewer: bool= False, 
                 setparamcb:Callable=default_parameter_setter,
                 timeout:Duration = Duration(seconds=1.0),
                 node : Node = None,
                 transitioncb:Callable=default_transitioncb, 
                 statecb:Callable=default_statecb
                 ):
        """
        
        """
        super().__init__(outcomes=[SUCCEED, ABORT,TIMEOUT],transitioncb=transitioncb,statecb=statecb)
        name = "etasl_"+task_name
        # if task is None:
        #     task = name

        #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues:
        # I am not so sure that the transition will fail if inappropriate, only when there is an error for an appropriate transition.
        self.add_state(name+".DEACTIVATE_ETASL", LifeCycle(transition=Transition.DEACTIVATE),
                transitions={SUCCEED: name+".CLEANUP_ETASL",
                            ABORT: name+".CLEANUP_ETASL",
                            TIMEOUT: ABORT}) 
        
        #This state is just added in case that etasl is already running. If not possible (ABORT) still the task continues
        self.add_state(name+".CLEANUP_ETASL", LifeCycle(transition=Transition.CLEANUP),
                transitions={SUCCEED: name+".PARAMETER_CONFIG",
                            ABORT: name+".PARAMETER_CONFIG",
                            TIMEOUT: ABORT}) 
        self.add_state(name+".PARAMETER_CONFIG", SetTaskParameters(
                                                    task_name=task_name,
                                                    srv_name = srv_name,
                                                    cb = setparamcb,
                                                    timeout=timeout,
                                                    node=node
                                                    ),
                       transitions={
                           SUCCEED: name+".CONFIG_ETASL"
                       })
        # self.add_state(name+".PARAMETER_CONFIG", ReadTaskParametersCB(task,setparamcb),
        #         transitions={SUCCEED: name+".ROBOT_SPECIFICATION",
        #                     ABORT: ABORT,
        #                     TIMEOUT: ABORT})

        # self.add_state(name+".ROBOT_SPECIFICATION", ReadRobotSpecificationFile(task),
        #         transitions={SUCCEED: name+".TASK_SPECIFICATION",
        #                     ABORT: ABORT,
        #                     TIMEOUT: ABORT})

        # self.add_state(name+".TASK_SPECIFICATION", ReadTaskSpecificationFile(name),
        #         transitions={SUCCEED: name+".CONFIG_ETASL",
        #                     ABORT: ABORT,
        #                     TIMEOUT: ABORT})

        self.add_state(name+".CONFIG_ETASL", LifeCycle(transition=Transition.CONFIGURE),
                transitions={SUCCEED: name+".ACTIVATE_ETASL",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        #state_name = f"RUNNING_{name}" 

        self.add_state(name+".ACTIVATE_ETASL", LifeCycle(transition=Transition.ACTIVATE),
                transitions={
                            #SUCCEED: name+".RUNNING",
                            ABORT: ABORT,
                            TIMEOUT: ABORT})

        # self.add_state(name+".RUNNING", Executing(name),
        #         transitions={"e_finished@etasl_node": SUCCEED,
        #                     ABORT: ABORT})

        #if display_in_viewer:
        #    YasminViewerPub('{} (nested FSM)'.format(name), self)

    
