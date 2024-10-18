# Yasmin_action
# Copyright (C) Erwin Aertbeliën, Santiago Iregui, 2024
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
# and some additional states for Yasmin to useful to experiment
# and demonstrate.


import threading
import time
import json
import jsonschema

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from yasmin_action_interfaces.action import YasminTask
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

from .yasmin_ticking_etasl import *

import rclpy 
# # still needed for ros_transition_cb
def set_logger(node):
    global logger
    logger=node.get_logger()

def get_logger():
    global logger
    return logger






# example to demonstrate that you can generate feedback from with a state:
# in a similar way we could write the result
class FeedbackState(State):
    """
    Example of a Yasmin State that publishes feedback of the action
    """
    def __init__(self) -> None:
        super().__init__(outcomes=["next"])

    def execute(self, blackboard: Blackboard) -> str:
        if "goal_handle" in blackboard:
            goal_handle=blackboard["goal_handle"]
            feedback_msg = YasminTask.Feedback()
            feedback_msg.state="feedback_state"
            feedback_msg.parameters=f"my_feedback {blackboard['counter']}"
            goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)
        return "next"

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
#         feedback_msg = YasminTask.Feedback()
#         feedback_msg.state="transition"
#         feedback_msg.parameters=f"transition {source}:{transition} -> {target}"
#         goal_handle.publish_feedback(feedback_msg)    
#     return


def action_state_cb(statemachine,blackboard,state):
    """
    returns the current state name and the contents as JSON of blackboard["feedback"] 
    """
    get_logger().info(f'entering state "{state}"')    
    if "goal_handle" in blackboard:
        goal_handle=blackboard["goal_handle"]
        feedback_msg = YasminTask.Feedback()
        feedback_msg.state=state
        if "feedback" in blackboard:
            try:
                feedback_msg.parameters=json.dumps(blackboard["feedback"])
            except TypeError as err:
               get_logger().error('blackboard["feedback"] contains data types that cannot be represented in JSON')
               feedback_msg.parameters="{}"
        else:
            feedback_msg.parameters="{}"        
        goal_handle.publish_feedback(feedback_msg)        




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
class CancelState(State):
    """
    Example Yasmin State that continuously checks is_cancel_requested()  of the action server
    """
    def __init__(self) -> None:
        super().__init__(outcomes=["next","cancel"])

    def execute(self, blackboard: Blackboard) -> str:
        for i in range(0,100):            
            if "goal_handle" in blackboard:
                goal_handle=blackboard["goal_handle"]
                if goal_handle.is_cancel_requested:
                    logger.info("CancelState observed is_cancel_requested")
                    return "cancel"
            time.sleep(0.01)
        return "next"






class EmptyStateMachine(StateMachine):
    def __init__(self):
        super().__init__(["   waiting_for_action   "])
        #self.add_state("WAITING",WaitingForAction(),transitions={})

class YasminActionServer(Node):
    """Action server that processes yasmin tasks one goal at a time.

    Some deep knowledge on YasminViewerPub is necessary to adapt it to another state machine
    See the main on how to use this.
    """

    def __init__(self,bm, statemachines,name="yasmin_action_server"):
        super().__init__(name)
        self.blackboard = bm 
        self.statemachines = statemachines
        self.empty_statemachine = EmptyStateMachine()
        self.cbg = ReentrantCallbackGroup()
        self.viewer = None
        self._current_goal_request_lock = threading.Lock()
        self._current_goal_request      = None
        self.input_parameters = {}        # decoded as (possibly recursive) Dict (No need for lock, action server makes sure only one thread is writing or reading)
        self._action_server = ActionServer(
            self,
            YasminTask,
            'yasmintask',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cbg)
        self.get_logger().info('YasminActionServer started')

    def set_viewer(self,viewer:YasminViewerPub):
        self.viewer = viewer
        if self.viewer is not None:
            self.viewer._fsm = self.empty_statemachine
            self.viewer._fsm_name = "waiting for action"

    def destroy(self):
        print("destroy")
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """called to accept or reject a client request."""
        # self.statemachines does not change during lifetime YasminActionServer
        # goal_request is local to this function

        # check existence of task
        self.get_logger().info(f'Received goal request for "{goal_request.task}"')
        if not (goal_request.task in self.statemachines):
            self.get_logger().info(f'Rejected goal request:  task "{goal_request.task}" is not known by the system')
            return GoalResponse.REJECT
        
        # decode input parameters
        try:
            param = json.loads(goal_request.parameters)
        except json.JSONDecodeError as err:
            self.get_logger().error(f"Rejected goal request: error decoding parameters of goal : {str(err)}")
            return GoalResponse.REJECT
        
        # validate parameters
        if hasattr(self.statemachines[goal_request.task],'input_parameters_schema'):
            try:
                jsonschema.validate(instance=param,
                                    schema=self.statemachines[goal_request.task].input_parameters_schema)
            except jsonschema.exceptions.ValidationError as err:
                self.get_logger().error( f'Rejected goal request: error validating parameters of task "{goal_request.task}" : {str(err)}')
                return GoalResponse.REJECT
            except jsonschema.exceptions.SchemaError as err:
                self.get_logger().error(f'Rejected goal request: error validating the schema of task "{goal_request.task}" : {str(err)}')
                return GoalResponse.REJECT
        # check concurrency        
        with self._current_goal_request_lock:
            if self._current_goal_request is not None:
                self.get_logger().error('Rejected goal request: another action is still running')
                return GoalResponse.REJECT
            self._current_goal_request = goal_request
        
        # only make changes to the YasminActiveServer instance when we are sure to be alone:
        self.get_logger().info('Accepted goal request for task "{goal_request.task}"')
        self.input_parameters = param 
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        "handle an accepted action"
        self.get_logger().info('handle accepted goal')        
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        self.blackboard["cancel_goal"] = True  # no need to further handle it in execute_callback, handled in 
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        try:
            self.blackboard["cancel_goal"] = False
            if goal_handle.is_cancel_requested:
                result = YasminTask.Result()
                result="cancel"
                return result
            self.blackboard["goal_handle"]      = goal_handle
            self.blackboard["input_parameters"] = self.input_parameters
            self.blackboard["task"]             = goal_handle.request.task
            self.get_logger().info('Executing state machine {goal_handle.request.request.task}')
            sm = self.statemachines[goal_handle.request.task]
            if self.viewer is not None:
                self.viewer._fsm = sm
                self.viewer._fsm_name = goal_handle.request.task
            rate = self.create_rate(100)
            sm.reset()
            outcome=sm(self.blackboard)
            while outcome==TICKING:
                rate.sleep()
                outcome=sm(self.blackboard)
            self.get_logger().info(f'Finished state machine {goal_handle.request.task} with outcome {outcome}')
            result = YasminTask.Result()
            if self.viewer is not None:
                self.viewer._start_publisher() # force a display of the end condition
            result.outcome=outcome   
            if "result" in self.blackboard:
                try:
                    result.parameters=json.dumps(self.blackboard["result"])
                except TypeError as err:
                    self.get_logger().error('blackboard["result"] contains data types that cannot be represented in JSON')
                    result.parameters="{}"
            else:
                result.parameters="{}"
            if outcome==SUCCEED:            
                goal_handle.succeed()
            elif outcome==CANCEL:            
                goal_handle.canceled()
            else:
                self.get_logger().error("state machine has an unexpected outcome {outcome}, action is canceled")
                goal_handle.canceled()            
            if self.viewer is not None:
                self.viewer._fsm = self.empty_statemachine
                self.viewer._fsm_name = "waiting for action"             
        finally:
            with self._current_goal_request_lock:
                self._current_goal_request = None
        return result

class MyMessage(Generator):
    """
    Message(msg) returns a State that displays a message
    """
    def __init__(self,msg) -> None:
        super().__init__("message",[SUCCEED,])
        self.msg = msg
    def co_execute(self,blackboard: Blackboard):
        my_node = YasminNode.get_instance()
        log = my_node.get_logger()
        log.info(f'Entering MyMessage : {self.msg}')
        yield SUCCEED

def main(args=None):

    rclpy.init(args=args)
    blackboard = Blackboard()

    # BUGFIX :executed second time, it starts from the previous execution.
    # fixed by calling reset() in the action!!
    # ath the highest level somebody has to reset() manually!
    sm1 = ConditionWhile(lambda bm : not bm["cancel_goal"], 
        Sequence("timer", children=[
            ("timer",TimedWait(Duration(seconds=5.0) ) ),
            ("hello",MyMessage("Timer went off!"))
        ])
    )
    statemachines = {"assembly":sm1}
    empty_statemachine = EmptyStateMachine()
    action_server = YasminActionServer(blackboard,statemachines)
    pub = YasminViewerPub("error", empty_statemachine,10,node=action_server)
    action_server.set_viewer(pub)
    set_logger(action_server)
    

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)    
    executor.spin()
    
    action_server.destroy()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
