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
from betfsm import (TICKING,SUCCEED, CANCEL, ABORT, Generator,GeneratorWithList,TickingState,Callable,AlwaysOutcome)
import signal
import math

def get_path_value(blackboard, path, default=None, delimiter='/'):
    """
    gets a value in the blackboard at the given path
    """
    keys = [k for k in path.split(delimiter) if k]
    current = blackboard
    for key in keys:
        if isinstance(current, dict) and key in current:
            current = current[key]
        else:
            return default
    return current

def set_path_value(blackboard, path, value, delimiter='/'):
    """
    sets a value in the blackboard at the given path
    """
    keys = [k for k in path.split(delimiter) if k]   
    current = blackboard
    for i, key in enumerate(keys[:-1]):
        if key not in current or not isinstance(current[key], dict):
            current[key] = {}
        current = current[key]
    if keys:
        current[keys[-1]] = value


class Ctrl_C_Handler:        
    def __init__(self,blackboard,pth:str=None, repeated=math.inf):
        """
        Configures the system SIGINT (Ctrl+C) handler to interact with a blackboard, where
        a variable will set to True if Ctrl-c is pressed

        Args:
            blackboard (dict): The nested dictionary structure acting as the blackboard.
            pth (str): The filesystem-style path (e.g., "/sys/stop_requested") 
                where the interrupt status is stored. If None, resets to default handler.
            repeated (int, None): number of times to repeat ctrl-c before KeyboardInterrupt is raised

        Example:
            # To start monitoring for Ctrl+C
            Ctrl_C_Handler(blackboard,"/cancelation/ctrl_c",repeated=3) 
        """                
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
    
    def __init__(self, name:str,  cb:Callable, nominal_sm:TickingState=None, cleanup_sm:TickingState=None) -> None:
        """
        Args:
            name (str): name of the BeTFSM node
            cb(Callable): callback with signature `def cb(blackboard)->bool`, true if cancel is triggered.
            nominal_sm (TickingState): nominal state (running while checking cancelation), if None, 
                                      a state is used  that always returns SUCCEED
            cleanup_sm (TickingState): cleanup state (running after cancelation to cleanup). if None,
                                      a state is used  that always returns CANCEL

        Note: 
            syntax lambda function in python :     f = lambda x: x==0
        
        Usage patterns:
            1) to check a whole tree continuously for cancelation, can be for the whole statemachine or a
               subtree of the state machine

                blackboard_ctrl_c_handler(blackboard, "/cancelation/ctrl_c")
                sm_overall = CheckCancel("cancelation_check",                     
                    lambda bb: get_path_value(bb,"/cancelation/ctrl_c"), 
                    sm,
                    cleanup_sm)

            2) with no statemachines specified it can be used a node in a state machine/behavior node 
               that returns CANCEL when there is a cancelation request and SUCCEED otherwise

        """
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


    def co_execute(self,blackboard):
        while True:
            if self.nominal:
                if self.cb(blackboard):
                    self.nominal=False
                    outcome = self.states[0]["state"].reset()
                else:
                    outcome = self.states[0]["state"](blackboard)
                    yield outcome
            else:
                outcome = self.states[1]["state"](blackboard)
                yield outcome
            

    # def co_execute(self,blackboard): 
    #     if self.maxcount!=-1:       
    #         for c in range(self.maxcount):
    #             outcome = self.state(blackboard)
    #             while outcome==TICKING:
    #                 yield TICKING
    #                 outcome = self.state(blackboard)
    #             if outcome!=SUCCEED:
    #                 yield outcome
    #         yield SUCCEED
    #     else:
    #         # forever:
    #         while True:
    #             outcome = self.state(blackboard)
    #             if outcome!=SUCCEED:
    #                 yield outcome