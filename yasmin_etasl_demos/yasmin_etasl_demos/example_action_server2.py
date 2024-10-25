# example_action_sever2
#
# An example of an action server with fine-grained control over when
# to listen to a cancel request.
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

from rclpy.executors import MultiThreadedExecutor
from yasmin_etasl.yasmin_ticking_etasl import *
from yasmin_etasl.logger import get_logger,set_logger
from yasmin_etasl.graphviz_visitor import *
from yasmin_etasl.yasmin_action_server import YasminActionServer
import rclpy 



# import some example state machines:
from . import sm_up_and_down as examples

def run_while_publishing( sm):
    """
    small state machine that executes `sm` while publishing graphviz to a topic /gz
    """

    #do_not_expand_types doesn't work for now...
    return Concurrent("concurrent",[
            sm,
        GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
        #GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[])
])


def main(args=None):

    rclpy.init(args=args)
    
    node = YasminTickingNode.get_instance("yasmin_action_server")


    set_logger("default",node.get_logger())
    #set_logger("service",node.get_logger())
    set_logger("state",node.get_logger())


    blackboard = {} #Blackboard()
    
    # load your tasks
    load_task_list("$[yasmin_etasl_demos]/tasks/my_tasks.json",blackboard)


    # which state-machine will be exeuted for which task name:
    statemachines={}
    
    statemachines["up_and_down"] =  run_while_publishing(examples.Up_and_down_with_parameters_checking_for_cancel(node) )
    # if you add additional member `input_parameters_schema` the action server will use this to validate the input:    
    statemachines["up_and_down"].input_parameters_schema=examples.my_schema
    action_server = YasminActionServer(blackboard,statemachines,100,node)

    # single or multi threaded executor does not matter here, only default callback group is used (which is mutually exclusive)
    executor = MultiThreadedExecutor()  
    executor.add_node(action_server.node)    
    executor.spin()
    
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
