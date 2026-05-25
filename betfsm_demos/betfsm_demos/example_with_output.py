#!/usr/bin/env python3

#
# Example with output of crospi tasks.
#

#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Erwin Aertbelien
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

import sys
from collections import deque
import rclpy
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSHistoryPolicy,QoSReliabilityPolicy,QoSLivelinessPolicy
from crospi_interfaces.msg import Output

from betfsm import (
    Sequence,  Message, Repeat,
    SUCCEED, TICKING, CANCEL, ABORT, TIMEOUT, NO_EVENT,
    TickingState,TickingStateMachine,GeneratorWithState,get_logger,set_logger,
    EventSequential, Ctrl_C_Condition, AlwaysOutcome,
    get_path_value, set_path_value,get_path_location,
    Blackboard
)
from betfsm_crospi import load_task_list, CrospiTask, CrospiDeactivate
from betfsm_ros import BeTFSMNode,ROSRunner,Node,Duration,LifeCycle,Transition


# QoS profile:

#   Reliability: RELIABLE
#   History (Depth): UNKNOWN
#   Durability: VOLATILE
#   Lifespan: Infinite
#   Deadline: Infinite
#   Liveliness: AUTOMATIC
#   Liveliness lease duration: Infinite

        # self.node = node
        # self.topic_name = topic_name
        # self.queue = deque(maxlen=queue_size)
        # self.lock = Lock()
        # qos = QoSProfile(
        #     history     = QoSHistoryPolicy.KEEP_LAST,
        #     depth       = 5,
        #     reliability = QoSReliabilityPolicy.RELIABLE,
        #     durability  = QoSDurabilityPolicy.VOLATILE
        # )
        # self.subscription = node.create_subscription( String,  topic_name, self._callback, qos  )
        # self.node.get_logger().info( f"TopicEventReceiver attached to {topic_name}" )


class CrospiOutput(GeneratorWithState):
    """
    Record output of Crospi in a topic while executing subtree
    """
    def __init__(
            self,
            name:str,
            topic: str,
            subtree: TickingState = None,
            queue_size: int = 1_000_000,
            path: str = "/output",            
            node: Node = None,
        ) -> None:
        """Record output of Crospi in a topic while executing subtree

        Parameters
        ----------
        name : str
            name of the node
        topic : str
            ROS2 topic to subscribe to, should be of message type Output
        subtree : TickingState, optional
            subtree to execute, None will be converted into AlwaysOutcome(SUCCEED), by default None            
        queue_size : int, optional
            maximum length of the buffer to record the data, by default 1_000_000
        path : str, optional
            path inside the blackboard, by default "/output"
        node : Node, optional
            ROS2 node, if None, BeTFSMNode.get_instance() will be used. by default None
        !!! warning
            - if externally interrupted, the deque will not be converted to a list
        """
        if subtree==None:
            subtree = AlwaysOutcome(SUCCEED)
        super().__init__("eTaSLOutput",[SUCCEED, CANCEL],subtree)
        if node==None:
            self.node = BeTFSMNode.get_instance()
        else:
            self.node = node
        self.topic = topic
        self.qos = QoSProfile(
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.VOLATILE
        )
        self.path       = path
        self.queue_size = queue_size

    def cb_msg(self,msg) -> None:
        if len(msg.is_declared)>0 and all(msg.is_declared):
            if self.first_time:
                self.header.append([n for n in msg.names ])
                self.first_time = False
            self.queue.append( [d for d in msg.data ])
            

    def co_execute(self,blackboard):
        path_base, path_key           = get_path_location(blackboard,self.path)
        self.queue                    = deque(maxlen=self.queue_size)
        self.header                   = []        
        path_base[path_key]           = dict()
        path_base[path_key]["data"]   = self.queue
        path_base[path_key]["header"] = self.header
        self.first_time               = True
        if path_base==None:
            get_logger("crospi").warn("CrospiOutput: blackboard path is not valid")
            return CANCEL
        subscription = self.node.create_subscription(Output,self.topic,self.cb_msg,self.qos)
        while True:
            outcome = self.state(blackboard)
            if outcome!=TICKING:
                break
            yield TICKING        
        self.node.destroy_subscription(subscription)
        #self.queue = list(self.queue)
        yield outcome




class MySequence(Sequence):
    def __init__(self):
        super().__init__("my_sequence", [
            CrospiTask("MovingHome","MovingHome"),
            CrospiTask("MovingDown","MovingDown"),
            CrospiTask("MovingUp","MovingUp"),
            CrospiOutput("output","/my_topic",CrospiTask("MovingSpline","MovingSpline"),queue_size=20 )
        ] )

# main
def main(args=None):
    rclpy.init(args=args)    
    my_node = BeTFSMNode.get_instance("example_with_output")

    set_logger("default",my_node.get_logger())
    set_logger("crospi",my_node.get_logger())
    
    blackboard = {}
    load_task_list("$[crospi_application_template]/skill_specifications/libraries/skill_lib_example/tasks/skill_example.json",blackboard)    

    # running MySequence() and cleaning up when CTRL_C is pressed        
    nominal_sm = Repeat("repeat",-1,MySequence())
    cleanup_sm = CrospiDeactivate(force_outcome=CANCEL)
    sm = EventSequential("check_cancel", Ctrl_C_Condition("CTRL_C",repeated=3),{NO_EVENT:nominal_sm, "CTRL_C":cleanup_sm})

    # ROSRunner accepts command-line parameters (see --help)
    # has many more optional arguments than used below, see API documentation
    runner = ROSRunner(my_node,sm,blackboard, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)

    try:
        runner.run()
    except KeyboardInterrupt:
        my_node.destroy_node()
        return   
    my_node.destroy_node()
    rclpy.shutdown()
    print("shutdown")

if __name__ == "__main__":
    sys.exit(main(sys.argv))
