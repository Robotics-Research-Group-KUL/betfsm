# betfsm_action
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


# Defines the action server
# and some additional states for BeTFSM to easily manage common tasks in an action server.
#


import threading
import time
import json
import jsonschema

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from betfsm_interfaces.action import Task

#from .betfsm_etasl import *
from .betfsm import *


from .logger import get_logger,set_logger

from .graphviz_visitor import *

import rclpy 






class FeedbackState(TickingState):
    """
    """
    def __init__(self, name:str, state:str, cb:Dict) -> None:
        """
        A ticking state that uses a callback function to publish to the feedback topic
        of an action.

        Parameters:
            name:
                name of the instance 
            state:
                is passed as the `state` field of the feedback message
            cb:
                callback function that returns a dictionary, possibly hierarchical that will 
                be mapped to the parameters **string** field of the feedback message.
                The callback has signature `def cb(bbb) -> Dict` 
        """
        super().__init__(name,outcomes=[SUCCEED])
        self.name = name
        self.cb = cb

    def execute(self, blackboard: Blackboard) -> str:
        if "goal_handle" in blackboard:
            goal_handle=blackboard["goal_handle"]
            feedback_msg = Task.Feedback()
            feedback_msg.state=self.name
            feedback_msg.parameters=json.dumps(self.cb(blackboard))
            goal_handle.publish_feedback(feedback_msg)
        return SUCCEED

# now using action_state_cb for more structured feedback
#
# def ros_transitioncb(statemachine,blackboard,source,transition,target):
#     """   
#     A callback function that generates feedback_msg with info on transitions.
#     (to be used with cbStateMachine, a replacement for StateMachine, see cb_state_machine.py)
#     """
#     global logger
#     logger.info("transition: %s:%s --> %s" % (source, transition, target))
#     if "goal_handle" in blackboard:
#         goal_handle=blackboard["goal_handle"]
#         feedback_msg = Task.Feedback()
#         feedback_msg.state="transition"
#         feedback_msg.parameters=f"transition {source}:{transition} -> {target}"
#         goal_handle.publish_feedback(feedback_msg)    
#     return


# def action_state_cb(statemachine,blackboard,state):
#     """
#     returns the current state name and the contents as JSON of blackboard["feedback"] 
#     """
#     get_logger().info(f'entering state "{state}"')    
#     if "goal_handle" in blackboard:
#         goal_handle=blackboard["goal_handle"]
#         feedback_msg = Task.Feedback()
#         feedback_msg.state=state
#         if "feedback" in blackboard:
#             try:
#                 feedback_msg.parameters=json.dumps(blackboard["feedback"])
#             except TypeError as err:
#                get_logger().error('blackboard["feedback"] contains data types that cannot be represented in JSON')
#                feedback_msg.parameters="{}"
#         else:
#             feedback_msg.parameters="{}"        
#         goal_handle.publish_feedback(feedback_msg)        




def check_cancel_transitioncb(cancel_outcome):
    """
    returns a handler to check whether cancel is requested of the ROS Action,
    This handler is handy when you want to be able to cancel after each transition in the state machine.
    Instead putting a CancelState() (see below) between each transtion, you just can add this callback function.
    In case of a cancel request it will transition to cancel_state (input parameter)

    parameters
    ----------
    cancel_outcome : transition will be come cancel_outcome when cancel was requested

    returns
    -------
    a method that can be used as a transition callback in a cb_state_machine:
    will have signature cb(statemachine,blackboard,source,transition)    

    see also
    --------
    default_transitioncb for help on the callback
    """ 
    def cb(statemachine,blackboard,source,outcome):
        if "goal_handle" in blackboard:
                goal_handle=blackboard["goal_handle"]
                if goal_handle.is_cancel_requested:
                    get_logger().info("CancelState observed is_cancel_requested")
                    del blackboard["goal_handle"] # we only listen once for an cancel_request during the duration of an action
                    return cancel_outcome
        return outcome
    return cb



# example of handling the cancel of an Action
# A state that polls until time out or cancel given
class CheckForCanceledAction(Generator):
    """
    Checks whether the Action that is running has received a cancel request.
    """
    def __init__(self,name:str) -> None:
        """
        Checks whether the Action that is running has received a cancel request.
        Returns SUCCEED if nothing received, returns CANCEL if something received

        Parameters:
            name:
                name of this state
        """
        super().__init__(name,outcomes=[SUCCEED,CANCEL])

    def co_execute(self, bb: Blackboard):
        if ("goal_handle" in bb) and bb["goal_handle"].is_cancel_requested:
            get_logger().info(f"{self.name} observed that the running action received a cancel request")            
            yield CANCEL
        else:
            yield SUCCEED

class WhileNotCanceled(GeneratorWithState):
    """
    Runs underlying state as long as not canceled. Returns outcome of state,
    yields CANCEL if there is a client side cancel request.
    """
    def __init__(self,name:str, state:TickingState) -> None:
        """
        Runs underlying state as long as not canceled. Returns outcome of state
        yields CANCEL if there is a client side cancel request.

        Parameters:
            name:
                name of this state
            state:
                underlying state
        """
        super().__init__(name,[CANCEL],state)
        

    def co_execute(self, bm: Blackboard):
        while not ("goal_handle" in bm and bm["goal_handle"].is_cancel_requested):
            outcome = self.state(bm)
            yield outcome        
        get_logger().info(f"{self.name} observed that the running action received a cancel request")          
        self.state.reset()  
        yield CANCEL
            

class BeTFSMActionServer:
    """Action server that processes BeTFSM tasks one goal at a time.  Each action starts state machine
    """

    def __init__(self,blackboard:Blackboard, statemachines,frequency:int = 100,node:Node=None):
        """
        Parameters:
            blackboard:
                blackboard to use when responding to actions.
            statemachines:
                a list of task names and state machines.
            frequency:
                frequency at which to run the state machine once an action is received.
            node:
                ROS2 node for the action server,  if None, the singleton BeTFSMNode.get_instance() will be used.
        """
        if node is None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        self.blackboard = blackboard
        self.statemachines = statemachines
        self.cbg = ReentrantCallbackGroup()
        self._current_goal_request_lock = threading.Lock()
        self._current_goal_request      = None
        self.input_parameters = {}        # decoded as (possibly recursive) Dict (No need for lock, action server makes sure only one thread is writing or reading)
        self._action_server = ActionServer(
            node,
            Task,
            'task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cbg)        
        get_logger().info('BeTFSMActionServer started')
        list = "Listening to tasks:"
        for k,v in self.statemachines.items():
            list = list + "\n" + f"task {k}"
            if hasattr(v,"input_parameters_schema"):
                list = list + "  with schema:\n"+json.dumps(v.input_parameters_schema, indent=True)
            else:
                list = list + " with no schema"
        get_logger().info( list)        



    def destroy(self):
        print("destroy")
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """called to accept or reject a client request."""
        # self.statemachines does not change during lifetime BeTFSMActionServer
        # goal_request is local to this function

        # check existence of task
        get_logger().info(f'Received goal request for "{goal_request.task}"')
        if not (goal_request.task in self.statemachines):
            get_logger().warning(f'Rejected goal request:  task "{goal_request.task}" is not known by the system')
            return GoalResponse.REJECT
        
        # decode input parameters
        try:
            param = json.loads(goal_request.parameters)
        except json.JSONDecodeError as err:
            get_logger().warning(f"Rejected goal request: error decoding parameters of goal : {str(err)}")
            return GoalResponse.REJECT
        
        # validate parameters
        if hasattr(self.statemachines[goal_request.task],'input_parameters_schema'):            
            try:
                jsonschema.validate(instance=param,
                                    schema=self.statemachines[goal_request.task].input_parameters_schema)
            except jsonschema.exceptions.ValidationError as err:
                get_logger().warning( f'Rejected goal request: error validating parameters of task "{goal_request.task}" : {str(err)}')
                return GoalResponse.REJECT
            except jsonschema.exceptions.SchemaError as err:
                get_logger().error(f'Rejected goal request: error validating the schema of task "{goal_request.task}" : {str(err)}')
                return GoalResponse.REJECT
        # check concurrency        
        with self._current_goal_request_lock:
            if self._current_goal_request is not None:
                get_logger().warning('Rejected goal request: another action is still running')
                return GoalResponse.REJECT
            self._current_goal_request = goal_request
        
        # only make changes to the BeTFSMActiveServer instance when we are sure to be alone:
        get_logger().info(f'Accepted goal request for task "{goal_request.task}"')
        self.input_parameters = param 
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        "handle an accepted action"
        get_logger().info('handle accepted goal')        
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        get_logger().info('Received cancel request')
        self.blackboard["cancel_goal"] = True  # no need to further handle it in execute_callback, handled in 
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        try:
            self.blackboard["cancel_goal"] = False
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()
            self.blackboard["goal_handle"]      = goal_handle
            self.blackboard["input_parameters"] = self.input_parameters
            self.blackboard["task"]             = goal_handle.request.task
            get_logger().info('Executing state machine {goal_handle.request.request.task}')
            sm = self.statemachines[goal_handle.request.task]
            rate = self.node.create_rate(100)
            sm.reset()
            outcome=sm(self.blackboard)
            while outcome==TICKING:
                rate.sleep()
                outcome=sm(self.blackboard)
            get_logger().info(f'Finished state machine {goal_handle.request.task} with outcome {outcome}')
            result = Task.Result()
            result.outcome=outcome   
            if "result" in self.blackboard:
                try:
                    result.parameters=json.dumps(self.blackboard["result"])
                except TypeError as err:
                    get_logger().error('blackboard["result"] contains data types that cannot be represented in JSON')
                    result.parameters="{}"
            else:
                result.parameters="{}"
            if outcome==SUCCEED:            
                goal_handle.succeed()
            elif outcome==CANCEL:    
                if goal_handle.is_cancel_requested:
                    get_logger().info(f"action is canceled upon request from client")
                    goal_handle.canceled()
                    result.outcome = CANCEL
                    result.parameters ='{"requested":true}'
                else:                            
                    get_logger().info(f"action is canceled by the state machine returning CANCEL")
                    goal_handle.abort()
                    result.outcome = CANCEL
                    result.parameters ='{"requested":false}'                   
            else:
                get_logger().error(f"state machine has an unexpected outcome {outcome}, action is canceled")
                goal_handle.abort()            
                result.outcome = CANCEL
                result.parameters =f'{{"requested":false, "unexpected_outcome":"{outcome}" }}' 
        finally:
            with self._current_goal_request_lock:
                self._current_goal_request = None
        get_logger().info(f'Task is finished with result {result}')
        return result



