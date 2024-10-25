# sm_up_and_down.py
#
# A simple demo statemachine that moves up and down.
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


from rclpy.duration import Duration
from yasmin_ros.yasmin_node import YasminNode
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import SUCCEED, ABORT,CANCEL,TIMEOUT

from yasmin_etasl.yasmin_ticking import *
from yasmin_etasl.yasmin_ticking_ros import *
from yasmin_etasl.yasmin_ticking_etasl import *
#from yasmin_etasl.graphviz_visitor import *


from yasmin_etasl.yasmin_action_server import CheckForCanceledAction

import math

#
#
# A state machine that moves home, up, down and up again
#
# Different versions of the same functionality:
#


def up_and_down_as_a_function(node=None):
    """
    """
    return  Sequence("my_sequence", children=[
                                        eTaSL_StateMachine("movinghome","MovingHome",node=node),
                                        eTaSL_StateMachine("movingup","MovingUp",node=node),
                                        eTaSL_StateMachine("movingdown","MovingDown",node=node),            
                                        eTaSL_StateMachine("movingup2","MovingUp",node=node),
                                        Message("Robot is finished")
                                    ] )


def up_and_down_as_a_function_and_a_timer(node=None):
    """
    """
    return ConcurrentSequence("up_and_down_as_a_function", children=[
            Sequence("my_sequence", children=[
                                        eTaSL_StateMachine("movinghome","MovingHome",node=node),
                                        eTaSL_StateMachine("movingup","MovingUp",node=node),
                                        eTaSL_StateMachine("movingdown","MovingDown",node=node),            
                                        eTaSL_StateMachine("movingup2","MovingUp",node=node),
                                        Message("Robot is finished")
                                    ] ),
            Sequence("timer", children=[
                                TimedWait("timed_wait",Duration(seconds=3.0),node=node ),
                                Message(msg="Timer went off!")
                              ] )
            ])

class Up_and_down_as_a_class(Sequence):
    """
    """
    def __init__(self,node=None):
        """
        """
        super().__init__("Up_and_down_as_a_class")
        self.add_state(eTaSL_StateMachine("movinghome","MovingHome",node=node))
        self.add_state(eTaSL_StateMachine("movingup","MovingUp",node=node))
        self.add_state(eTaSL_StateMachine("movingdown","MovingDown",node=node))
        self.add_state(eTaSL_StateMachine("movingup2","MovingUp",node=node))
        self.add_state(Message("Hello world"))




class Up_and_down_with_parameters(Sequence):
    def __init__(self,node):
        """
        """
        super().__init__("Up_and_down_as_a_class")

        class MyComputations(Generator):
            def __init__(self):
                super().__init__("MyComputations",[SUCCEED])
            def co_execute(self,bb):
                bb["home_computations"]={}
                bb["home_computations"]["joint_1"] = bb["output"]["home1"]["jpos1"]*180.0/math.pi +100.0
                yield SUCCEED 

        def my_parameters(bb:Blackboard)->Dict:
            result = {}
            result["joint_1"] = bb["home_computations"]["joint_1"]
            result["joint_2"] = bb["output"]["home1"]["jpos2"]*180.0/math.pi+5
            return result
        
        self.add_state(eTaSL_StateMachine("home1","MovingHome" ,node=node))
        self.add_state(LogBlackboard("LogBlackboard1",["output"]))
        self.add_state( MyComputations() )
        self.add_state(LogBlackboard("LogBlackboard1",["home_computations"]))
        self.add_state(eTaSL_StateMachine("home2","MovingHome", cb=my_parameters, node=node) )
        self.add_state(eTaSL_StateMachine("home3","MovingHome",node=node))
        self.add_state(LogBlackboard("Logblackboard2",["output"] ))
        self.add_state(eTaSL_StateMachine("movingup","MovingUp",node=node),)
        self.add_state(eTaSL_StateMachine("movingdown","MovingDown",node=node))
        self.add_state(eTaSL_StateMachine("movingup2","MovingUp",node=node))
        self.add_state( Message(msg="Hello world") )



class Up_and_down_with_parameters_lambda(Sequence):
    def __init__(self, node):
        """
        """
        super().__init__("Up_and_down_with_parameters_lambda")
        
        self.add_state(eTaSL_StateMachine("home1","MovingHome",node=node))
        self.add_state(LogBlackboard("LogBlackboard1",["output"]))
        self.add_state(Compute("compute_state",["home_computations"], lambda bb: {"joint_1" : bb["output"]["home1"]["jpos1"]*180.0/math.pi + 100.0} ))
        self.add_state(LogBlackboard("LogBlackboard1",["home_computations"]))                                                   
        self.add_state(eTaSL_StateMachine("home2","MovingHome",
                                            cb=lambda bb: {"joint_1": bb["home_computations"]["joint_1"],
                                                           "joint_2": bb["output"]["home1"]["jpos2"]*180.0/math.pi+5,
                                                          },
                                            node=node) )
        self.add_state(eTaSL_StateMachine("home3","MovingHome",node=node))
        self.add_state(LogBlackboard("Logblackboard2",["output"] ))
        self.add_state(eTaSL_StateMachine("movingup","MovingUp",node=node),)
        self.add_state(eTaSL_StateMachine("movingdown","MovingDown",node=node))
        self.add_state(eTaSL_StateMachine("movingup2","MovingUp",node=node))
        self.add_state( Message(msg="Hello world") )



class Up_and_down_with_parameters_checking_for_cancel(Sequence):
    def __init__(self, node):
        """
        """
        super().__init__("Up_and_down_with_parameters_checking_for_cancel")
        
        self.add_state(eTaSL_StateMachine("home1","MovingHome",node=node))
        self.add_state(LogBlackboard("LogBlackboard1",["output"]))
        self.add_state(Compute("compute_state",["home_computations"], lambda bb: {"joint_1" : bb["output"]["home1"]["jpos1"]*180.0/math.pi + 100.0} ))
        self.add_state(LogBlackboard("LogBlackboard1",["home_computations"]))                                                   
        self.add_state(eTaSL_StateMachine("home2","MovingHome",
                                            cb=lambda bb: {"joint_1": bb["home_computations"]["joint_1"],
                                                           "joint_2": bb["output"]["home1"]["jpos2"]*180.0/math.pi+5,
                                                          },
                                            node=node) )
        self.add_state(eTaSL_StateMachine("home3","MovingHome",node=node))
        self.add_state(LogBlackboard("Logblackboard2",["output"] ))
        # looping 10 times and checking for cancelled action at a specific point in the
        # task. A CANCEL outcome will fall through the states "up_and_down_cycle","repeat","
        # Up_and_down_as_a_class"
        self.add_state(Repeat("repeat",10,
                Sequence("up_and_down_cycle",
                        [
                            eTaSL_StateMachine("movingup","MovingUp",node=node,cb= lambda bb: {"delta_z": bb["input_parameters"]["delta_z"]}),
                            eTaSL_StateMachine("movingdown","MovingDown",node=node,cb= lambda bb: {"delta_z": -bb["input_parameters"]["delta_z"]}),
                            CheckForCanceledAction("check_for_cancel")
                        ]
                )
        ))       

#static member:    
my_schema=json.loads("""
        {
            "title": "upanddown",
            "description": "task that moves up and down",
            "type": "object",
            "properties" : {
                "delta_z" : { "type" : "number", "description": "height to move up and down" }
            },
            "required" : [ "delta_z"],
            "additionalProperties" : false
        }
    """)