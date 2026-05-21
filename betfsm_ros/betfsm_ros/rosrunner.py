# betfsmrunnergui.py 
#
# Copyright (C) Erwin Aertbeliën, 2025,2026
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



from betfsm.betfsm import TickingState,Blackboard,TICKING
from betfsm import get_logger,set_loggers_from_specification_string, RunnerBase
import sys
from rclpy.clock import Clock, ClockType
from betfsm import Lock
from rclpy.node import Node
import rclpy
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
import time

from betfsm import RunnerBase



class ROSRunner(RunnerBase):
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.

    This is multi-threaded, only access member variables using the methods designed for this. (i.e. set_outcome, get_outcome)
    """    
    def __init__(self, node:Node, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency:float=5,debug:bool=False, 
                 display_active:bool=False, betfsm_log:str=None,name_filter:str="", type_filter:str="",
                 allow_generate_dot:bool=True, allow_generate_sm_dot:bool=True,allow_generate_json:bool=True,serve:bool=True,
                 host:str="0.0.0.0", port:str=8000, workers:int=1, log_level:str="info"):
   
        super().__init__(statemachine, blackboard, frequency, publish_frequency, debug, display_active, betfsm_log,
                         name_filter,type_filter,allow_generate_dot,allow_generate_sm_dot,allow_generate_json,serve,
                         host, port, workers, log_level)
        args = self.args

        # ROS dependend: everything timer related, logger, node, clocks, synchronization primitives:
        self.frequency       = args.frequency
        self.interval_sec    = 1.0/args.frequency
        self.interval_ns     = int(1.0/args.frequency*1E9)
        self.publish_period  = int(1.0/args.publish_frequency*1E9)  # in nanoseconds  
        set_loggers_from_specification_string(args.betfsm_log, node.get_logger())
        self.shutdown_future = Future()
        self.node            = node
        self.steady_clock    = Clock(clock_type=ClockType.STEADY_TIME)  #steady_now = steady_clock.now()
        self.timer           = self.node.create_timer(self.interval_sec, self.timer_cb)
        self.outcome_lock = Lock()
        self.first_time = True                        

    def parse_arguments(self, parser, group_app, group_uvr):
        clean_args        = remove_ros_args(args=sys.argv[1:])
        args              = parser.parse_args(clean_args)        
        return args

    def timer_cb(self):
        """ 
        called by ROS timer, should not be called by user.  
        """
        # both time sources should work.
        self.now = self.steady_clock.now().nanoseconds    
        #self.now = time.monotonic()*1.0E9        
        if self.first_time:
            self.previous_run = self.now - self.interval_ns  
            self.next_publish = self.now  # ensure publishing at start
            self.initialize()
            if self.debug:
                get_logger().info(f"ROSRunner: started at {self.now/1.0E9:10.4f} s started (frequency:{self.frequency})")        
            self.first_time = False
        jitter = (self.now - self.previous_run)*1E-9 - self.interval_sec
        if abs(jitter) > self.interval_ns*0.5:   
            get_logger().warn(f"Timing: large deviation : {jitter:6f} s")   
        if self.debug:
            self.stats.add(jitter)           
        outcome = self.statemachine(self.blackboard)
        if (self.now >= self.next_publish - self.interval_ns*0.5) or (outcome != TICKING):
            self.process_publish_cycle()
            self.next_publish = self.now + self.publish_period
        if outcome!=TICKING:
            self.timer.cancel()            
            self.set_outcome(outcome)
            self.finalize()
            self.shutdown_future.set_result(True)         
        self.previous_run = self.now

    def run(self) -> str:
        """
        Runs the statemachine until it returns an outcome different from TICKING

        Refers to absolute time to avoid drifting. Uses monotonic clock to 
        avoid problems with system time changes.

        Returns:
            the final outcome of the statemachine        
        """        
        self.first_time = True
        rclpy.spin_until_future_complete(self.node, self.shutdown_future, executor=None,timeout_sec=None)
        return self.get_outcome()
        

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

