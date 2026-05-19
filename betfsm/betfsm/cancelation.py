# cancelation.py
#
# Managing signals that need to interrupt a BeTFSM tree, such as ctrl-c interrupt.
#
#
# Copyright (C) Erwin Aertbeliën, 2026
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

from betfsm.logger import *
from betfsm import (SUCCEED, CANCEL,
                    GeneratorWithList,TickingState,Callable,AlwaysOutcome,set_path_value,get_path_value,
                    deprecated_msg
                    )
import signal
import math



class Ctrl_C_Handler:        
    def __init__(self,blackboard,pth:str=None, repeated=math.inf):
        """
        Configures the system SIGINT (Ctrl+C) handler to interact with a blackboard, where
        a variable will set to True if Ctrl-c is pressed

        Arguments:
            blackboard (dict): 
                The nested dictionary structure acting as the blackboard.
            pth (str): 
                The filesystem-style path (e.g., "/sys/stop_requested")  where the interrupt status 
                is stored. If None, resets to default handler.
            repeated (int, None): number of times to repeat ctrl-c before KeyboardInterrupt is raised

        Example:
            To start monitoring for Ctrl+C.  If ctrl-c is pressed, at the given path in the blackboard,
            a `True` value will be written.
            ```
            Ctrl_C_Handler(blackboard,"/cancelation/ctrl_c",repeated=3) 
            ```
        """                
        deprecated_msg("Use Ctrl_C_Receiver with CheckSequential/CheckOutcome instead")
        self.blackboard = blackboard
        self.pth        = pth
        self.repeated   = repeated
        self.count      = 0
        if self.pth is None:
            signal.signal(signal.SIGINT, signal.default_int_handler)
        else:
            set_path_value(self.blackboard,self.pth,False)
            signal.signal(signal.SIGINT, self.handler )
        

    def handler(self,signum, frame):
        set_path_value(self.blackboard,self.pth, True)
        self.count = self.count+1
        get_logger().info(f"Ctrl-C pressed {self.count} times, pressing {self.repeated} times will interrupt")            
        if self.count >= self.repeated:
            raise KeyboardInterrupt
    



class CheckCancel(GeneratorWithList):
    """
    State that contiuously evaluates a condition and if this condition is False it triggers the nominal state.
    Once the condition is True, the state switches to an abnormal state and will trigger
    the cleanup state (and will never go back to the normal state)
    """    
    def __init__(self, name:str,  cb:Callable, nominal_sm:TickingState=None, cleanup_sm:TickingState=None) -> None:
        """
        !!! warning
            **Deprecated, use Ctrl_C_Receiver with CheckSequential/Checkoutcome instead**
            
        Args:
            name (str): name of the BeTFSM node
            cb(Callable): callback with signature `def cb(blackboard)->bool`, true if cancel is triggered.
            nominal_sm (TickingState): nominal state (running while checking cancelation), if None, 
                                      a state is used  that always returns SUCCEED
            cleanup_sm (TickingState): cleanup state (running after cancelation to cleanup). if None,
                                      a state is used  that always returns CANCEL

        Remarks: 
          - syntax lambda function in python :     `f = lambda bb: get_path_value(bb,"/mypath/...")`, to examine
            the blackboard `get_path_value` can be used, but you can write other functions to check for a cancel
            condition

          - Can be used to check for a ctrl-c condition, but is not limited to that, can check all kinds
            of interrupt conditions such as events coming from a gui, or button,etc..
            
          - to check a whole tree continuously for cancelation, can be for the whole statemachine or a
               subtree of the state machine

                ```
                blackboard_ctrl_c_handler(blackboard, "/cancelation/ctrl_c")
                sm_overall = CheckCancel("cancelation_check",                     
                    lambda bb: get_path_value(bb,"/cancelation/ctrl_c"), 
                    sm,
                    cleanup_sm)
                ```

          - with no statemachines specified it can be used a node in a state machine/behavior node 
               that returns `CANCEL` when there is a cancelation request and `SUCCEED` otherwise

        """
        deprecated_msg("Use Ctrl_C_Receiver with CheckSequential/CheckOutcome instead")
        super().__init__(name,[])
        self.cb = cb
        self.nominal = True

        # used GeneratorWithList for cleanup
        if nominal_sm is None:
            nominal_sm = AlwaysOutcome("nominal",SUCCEED)
        self.add_state(nominal_sm)
        if cleanup_sm is None:
            cleanup_sm = AlwaysOutcome("cleanup",CANCEL)
        self.add_state(cleanup_sm)

    def reset(self):
        self.nominal = True
        return super().reset()
    
    def co_execute(self,blackboard):
        while True:
            if self.nominal:
                if self.cb(blackboard):
                    self.nominal=False
                    self.states[0]["state"].reset()
                else:
                    outcome = self.states[0]["state"](blackboard)
                    yield outcome
            else:
                outcome = self.states[1]["state"](blackboard)
                yield outcome
            