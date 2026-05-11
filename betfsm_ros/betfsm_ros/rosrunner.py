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


from betfsm import RunnerBase



class ROSRunner(RunnerBase):
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.

    This is multi-threaded, only access member variables using the methods designed for this. (i.e. set_outcome, get_outcome)
    """    
    def __init__(self, node:Node, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency=5,debug: bool = False, 
                 display_active=False, betfsm_log=None,name_filter="", allow_generate_dot=True,allow_generate_json=True,serve=True,
                 host="0.0.0.0", port=8000, workers=1, log_level="info"):
   
        super().__init__(statemachine, blackboard, frequency, publish_frequency, debug, display_active, betfsm_log,name_filter,allow_generate_dot,allow_generate_json,serve,
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

    def parse_arguments(self, parser, group_app, group_uvr):
        clean_args        = remove_ros_args(args=sys.argv[1:])
        args              = parser.parse_args(clean_args)        
        return args

    def timer_cb(self):
        """ 
        called by ROS timer, should not be called by user.  
        """
        self.now = self.steady_clock.now().nanoseconds            
        if self.first_time:
            self.next_run = self.now
            self.next_publish = self.now  # ensure publishing at start
            self.initialize()
            if self.debug:
                get_logger().info(f"BeTFSMROSRunnerGUI time: {self.now/1.0E9:10.4f} s started (frequency:{self.frequency})")        
            self.first_time = False
        if self.now > self.next_run + self.interval_ns*0.5:
            self.next_run = self.now - self.interval_ns
            get_logger().warn(f"Sample period exceeded/drifted : {(self.now-self.next_run)/1E9:.6f} s late")   
        if self.debug:
            self.stats.add((self.now - self.next_run)/1.0E9)
        self.next_run += self.interval_ns            
        outcome = self.statemachine(self.blackboard)
        if (self.now >= self.next_publish - self.interval_ns/2) or (outcome != TICKING):
            self.process_publish_cycle()
            self.next_publish = self.now + self.publish_period
        if outcome!=TICKING:
            self.timer.cancel()            
            self.set_outcome(outcome)
            get_logger().info(f"BeTFSM finished with outcome {outcome}")
            self.shutdown_future.set_result(True)         

    def run(self):
        """
        Runs the statemachine until it returns an outcome different from TICKING

        Refers to absolute time to avoid drifting. Uses monotonic clock to 
        avoid problems with system time changes.

        Returns:
            the final outcome of the statemachine        
        """        
        self.first_time = True
        rclpy.spin_until_future_complete(self.node, self.shutdown_future, executor=None,timeout_sec=None)
        

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

