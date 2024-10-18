# sm_up_and_down.py
#
# A simple demo statemachine that moves up and down.
#
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



def up_and_down_as_a_function():
    return ConcurrentSequence("up_and_down_as_a_function", children=[
            ("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("MovingHome") ),
                        ("movingup",eTaSL_StateMachine("MovingUp") ),
                        ("movingdown",eTaSL_StateMachine("MovingDown") ),            
                        ("movingup",eTaSL_StateMachine("MovingUp")),
                        ("my_message",Message("Robot is finished"))
                      ]) 
            ),
            ("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0) ) ),
                        ("hello",Message("Timer went off!"))
                    ])
            )
    ])


class Up_and_down_as_a_class(ConcurrentSequence):
    def __init__(self):
        super().__init__("Up_and_down_as_a_class")
        self.add_state("task1", Sequence("my_sequence", children=[
                        ("movinghome",eTaSL_StateMachine("MovingHome") ),
                        ("movingup",eTaSL_StateMachine("MovingUp") ),
                        ("movingdown",eTaSL_StateMachine("MovingDown") ),            
                        ("movingup",eTaSL_StateMachine("MovingUp")),
                        ("my_message",Message("Hello world"))
                      ]))
        self.add_state("task2",Sequence("timer", children=[
                        ("timer",TimedWait(Duration(seconds=3.0) ) ),
                        ("hello",Message("Timer went off!"))
                    ]))
        
        