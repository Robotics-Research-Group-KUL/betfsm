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

from .yasmin_ticking import *
from .yasmin_ticking_ros import *
from .yasmin_ticking_etasl import *
from .graphviz_visitor import *


#
#
# A state machine that moves home, up, down and up again
#
# Different versions of the same functionality:
#


def up_and_down_as_a_function(node=None):
    """
    """
    return ConcurrentSequence("up_and_down_as_a_function", children=[
            ("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("movinghome","MovingHome",node=node) ),
                        ("movingup",eTaSL_StateMachine("movingup","MovingUp",node=node) ),
                        ("movingdown",eTaSL_StateMachine("movingdown","MovingDown",node=node) ),            
                        ("movingup",eTaSL_StateMachine("movingup2","MovingUp",node=node)),
                        ("my_message",Message("Robot is finished"))
                      ]) 
            ),
            ("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0),node=node ) ),
                        ("hello",Message("Timer went off!"))
                    ])
            )
    ])


class Up_and_down_as_a_class(ConcurrentSequence):
    def __init__(self,node=None):
        """
        """
        super().__init__("Up_and_down_as_a_class")

        self.add_state("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("movinghome","MovingHome",node=node) ),
                        ("movingup",eTaSL_StateMachine("movingup","MovingUp",node=node) ),
                        ("movingdown",eTaSL_StateMachine("movingdown""MovingDown",node=node) ),            
                        ("movingup",eTaSL_StateMachine("movingup2","MovingUp",node=node)),
                        ("my_message",Message("Hello world"))
                      ]))
        
        self.add_state("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0),node=node ) ),
                        ("hello",Message("Timer went off!"))
                    ]))

class Up_and_down_with_parameters(ConcurrentSequence):
    """
    Examples of showing different styles of setting parameters and doing computations.

    Remember: computations done in the constructor of this class are only executed during construction 
    of the statemachine, not during execution! This is the reason behind the use of additional state MyComputations,
    the function MyComputations.  A lambda function can be used to consisely do a small computations.
    """
    def __init__(self, node):
        """
        """
        super().__init__("Up_and_down_as_a_class")


        class MyComputations(Generator):
            def __init__(self):
                super().__init__("MyComputations",[SUCCEED])
            def co_execute(self,bm):
                bm["home_computations"]={}
                bm["home_computations"]["joint_1"] = -10
                yield SUCCEED 

        def my_parameters(bb:Blackboard)->Dict:
            return {"joint_1":0,"joint_2":-90}
        
        self.add_state("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("home1","MovingHome",cb=my_parameters ,node=node) ),
                        ("debug",LogBlackboard(["output"])),
                        ('compute', MyComputations()),
                        ("movinghome",eTaSL_StateMachine("home2","MovingHome",
                                                        cb=lambda bb: {"joint_1": bb["home_computations"]["joint_1"]+20} ,
                                                        node=node) ),
                        ("movinghome",eTaSL_StateMachine("home3","MovingHome",node=node) ),
                        ("debug",LogBlackboard(["output"])),
                        ("movingup",eTaSL_StateMachine("movingup","MovingUp",node=node) ),
                        ("movingdown",eTaSL_StateMachine("movingdown","MovingDown",node=node) ),            
                        ("movingup",eTaSL_StateMachine("movingup2","MovingUp",node=node)),
                        ("my_message",Message("Hello world"))
                      ]))
        self.add_state("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0) ) ),
                        ("hello",Message("Timer went off!"))
                    ]))

