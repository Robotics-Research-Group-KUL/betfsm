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



import uvicorn
import threading
import argparse
#import time
from betfsm.backend.app import app,publish_tick,set_webserver_param
from betfsm.betfsm import TickingState,Blackboard,TICKING, get_logger_categories
from betfsm import get_logger, set_loggers_from_specification_string_v2
from betfsm.graphviz_visitor import to_graphviz_dotfile
from betfsm.jsonvisitor import JsonVisitor, parse_name_filter
import json
import sys
from rclpy.clock import Clock, ClockType
from betfsm import Lock
from rclpy.node import Node
import rclpy
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
import signal
import math


class TimerStats:
    def __init__(self):
        self.reset();
    
    def reset(self):
        self.sum   = 0
        self.N     = 0
        self.sumsq = 0
        self.min   = math.inf
        self.max   = -math.inf
    
    def add(self, value):
        self.N     += 1
        self.sum   += value
        self.sumsq += value*value
        self.min   = min(self.min, value)
        self.max   = max(self.max, value)

    def __str__(self):
        mean = self.sum / self.N
        var  = (self.sumsq - self.sum*self.sum/self.N)/self.N
        std  =  math.sqrt(var)
        return f"mean = {mean:10.6f}, std = {std:10.6f}, min = {self.min:10.6f}, max = {self.max:10.6f}"
    
class BeTFSMRosRunnerGUI:
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.

    This is multi-threaded, only access member variables using the methods designed for this. (i.e. set_outcome, get_outcome)
    """
    def __init__(self, node:Node, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency=10,debug: bool = False, 
                 display_active=False, betfsm_log=None,name_filter="", allow_generate_dot=True,allow_generate_json=True,serve=True,
                 host="0.0.0.0", port=8000, workers=1, log_level="info"):
        """
        Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
        runs in the main thread.  The parameters frequency, publish_frequency, debug, display_active,
        and serve can be overridden by command-line parameters.  Use "--help" to show all command-line
        parameters.

        Parameters:
            node:
                Ros node to be used to define the timed loop for the runner
            statemachine:
                the TickingStateMachine to be run
            blackboard:
                the blackboard to be used
            frequency:
                frequency at which the statemachine is ticked (in Hz) (default=100Hz)
            publish_frequency:
                frequency at which to publish to the webbrowser (default= 10Hz)
            debug:
                If true outputs debug info on console each tick. (default=False)
            display_active:
                displays all active nodes on log at publish_frequency (only if changed!)
            betfsm_log:
                A list of categories, separated by ':' that will be logged, known categories
                are default, state, service,... But users and libraries can add their own.
            name_filter:
                comma-separated list of names of nodes that are not descended into in
                the graphical user interface
            allow_generate_dot:
                adds the generate_dot command-line parameter. [default: True]
            allow_generate_json:
                adds the generate_json command-line parameter. [default: True]
            serve:A : {active}
                starts webserver if True [default: True]
            host:
                the host IP of the network interface to bind to.  Default=0.0.0.0. 
                Typical values are:
                - 127.0.0.1 for only local acces and high security, for local development or production behind reverse proxy. 
                - localhost usually 127.0.0.1
                - 0.0.0.0 for anyone who knows your IP, lower security, for Docker, VM's or public access
                - 192.168.x.x for only your local network
            port:
                bind socket of server to this port, default=8000
            workers:
                number of worker processes, default=1
            log-level:
                log-level of the web-server (not BeTFSM), default = "info"
                choices are "critical", "error", "warning", "info", "debug", "trace"

        """
        self.statemachine = statemachine
        self.blackboard = blackboard
        epilog="""
        You can still pass ROS2 arguments according to ROS2 conventions: e.g.
        adding : "--ros-args --log-level debug" to your arguments
        """
        parser = argparse.ArgumentParser(description="BeTFSMRunnerGUI command line options",epilog=epilog)

        # --- 1. YOUR Custom Arguments ---
        group_app = parser.add_argument_group("BeTFSMRunnerGUI Options")
        group_app.add_argument("--frequency",type=float, default=frequency, help=f"frequency at which BeTFSM runs [default:{frequency}]")
        group_app.add_argument("--publish_frequency",type=float, default=publish_frequency, help=f"publishing frequency for GUI [default:{publish_frequency} ]")
        group_app.add_argument("--debug", action=argparse.BooleanOptionalAction, default=debug,help=f"Log statistics of the timing of each tick at publish_frequency [default: {debug}]")
        group_app.add_argument("--display_active",action=argparse.BooleanOptionalAction,default=display_active, help=f"Log the active nodes at rate equal to publish_frequency, only logs changes to activity[default: {display_active}] ")
        group_app.add_argument("--betfsm_log",type=str,default=betfsm_log, help=f"BeTFSM Log specification string, i.e. a list of categories separated with ':'. Known categories are {get_logger_categories()} but there can be user-defined categories   [default: '{betfsm_log}'] ")
        group_app.add_argument("--name-filter",type=str,default=name_filter, help=f"specifies a comma-separated list of naes (can be regular expressions) to filter out.[default: '{name_filter}'")
        if allow_generate_dot:
            group_app.add_argument("--generate_dot",type=str, default="", help="generate a graphviz .dot file from the state machine and store in the specified file (and quit the program without running)")
        if allow_generate_json:
            group_app.add_argument("--generate_json",type=str, default="", help="generate a json file from the state machine and store in the specified file (and quit the program without running)")
 
        group_app.add_argument("--serve",action=argparse.BooleanOptionalAction,default=serve,help=f"Start-up server with graphical user interface [default:{serve}]")

        # --- 2. UVICORN Specific Arguments ---
        # We use the exact names uvicorn.run() expects as kwargs
        group_uvr = parser.add_argument_group("Uvicorn Web Server Options")
        group_uvr.add_argument("--host", type=str, default=host, help=f"Bind socket to this host [default: {host}]")
        group_uvr.add_argument("--port", type=int, default=port, help=f"Bind socket to this port [default: {port}]")
        group_uvr.add_argument("--workers", type=int, default=workers, help=f"Number of worker processes[default: {workers}]")
        group_uvr.add_argument("--log-level", type=str, default=log_level, 
                               choices=['critical', 'error', 'warning', 'info', 'debug', 'trace'], 
                               help=f"log-level of the web-server [default: {log_level} ]")
        clean_args = remove_ros_args(args=sys.argv[1:])
        args = parser.parse_args(clean_args)

        if allow_generate_dot:
            if args.generate_dot:
                to_graphviz_dotfile(args.generate_dot, statemachine)
                print(f"graphviz file '{args.generate_dot}' is generated, and program will be terminated")
                sys.exit()
        if allow_generate_json:
            if args.generate_json:
                visitor = JsonVisitor()
                statemachine.accept(visitor)
                with open(args.generate_json,"w") as f:
                    json.dump(visitor.result(), f, indent=4)
                sys.exit()
        # 3. Pass arguments to uvicorn.run()
        # We convert the namespace to a dict and extract only what Uvicorn needs
        uvicorn_kwargs = {
            "host": args.host,
            "port": args.port,
            "workers": args.workers,
            "log_level": args.log_level,
        }

        # 4. set parameters for BeTFSMRunner:
        self.frequency = args.frequency
        self.interval_sec = 1.0/args.frequency
        self.interval_ns  = int(1.0/args.frequency*1E9)
        self.debug = args.debug
        self.display_active = args.display_active
        self.publish_period = int(1.0/args.publish_frequency*1E9)  # in nanoseconds  
        self.serve = args.serve
        #set_loggers_from_specification_string(args.betfsm_log)
        set_loggers_from_specification_string_v2(args.betfsm_log, node.get_logger())

        # 5. local variables of the run() that now have to be stored in the class and create ROS2 timer
        self.shutdown_future = Future()
        self.node = node
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)  #steady_now = steady_clock.now()
        self.first_time = True
        self.timer = self.node.create_timer(self.interval_sec, self.timer_cb)
        self.outcome = "TICKING"
        self.outcome_lock = Lock()
        self.previous_active = []

        get_logger().info(f"BeTFSMRunnerGUI parameters: {args}")
        if self.serve:
            self.name_filter = parse_name_filter(args.name_filter)
            self.type_filter = [] 
            set_webserver_param(statemachine,self.name_filter,self.type_filter)  # set parameters for web-app
            threading.Thread(
                target=lambda: uvicorn.run(app, **uvicorn_kwargs),
                daemon=True
            ).start()
        if self.debug:
            self.stats = TimerStats()

    def timer_cb(self):
        self.now = self.steady_clock.now().nanoseconds            
        if self.first_time:
            self.next_run = self.now
            self.next_publish = self.now - self.publish_period  # ensure publishing at start
            if self.serve or self.display_active:
                TickingState.global_publish_log = {}  # turn on global_publish_log
            if self.debug:
                get_logger().debug(f"BeTFSMROSRunnerGUI time: {self.now/1.0E9:10.4f} s started (frequency:{self.frequency})")
            self.set_outcome("TICKING")
            self.first_time = False
        if self.now > self.next_run + self.interval_ns*0.5:
            self.next_run = self.now - self.interval_ns
            get_logger().warn(f"Sample period exceeded/drifted : {(self.now-self.next_run)/1E9:.6f} s late")   
        if self.debug:
            self.stats.add((self.now - self.next_run)/1E9)
        self.next_run += self.interval_ns            
        outcome = self.statemachine(self.blackboard)
        if (self.now >= self.next_publish) or (outcome != TICKING):
            # ensure the last one is always published
            if self.serve:
                publish_tick() # publishes all states that where active between calls to publish_tick
            if self.display_active:
                gl=TickingState.get_global_log()
                if gl != self.previous_active:
                    active = "("
                    for k,v in TickingState.get_global_log().items():
                        active = active + v.name + " "
                    active=active+")"
                    get_logger().info(f"ACTIVE : {active}")
                    self.previous_active = gl.copy()
            else:
                active=""                
            if TickingState.global_publish_log is not None:
                TickingState.global_publish_log.clear()
            if self.debug:
                get_logger().info(f"Timer statistics : {self.stats}")
                self.stats.reset()
            self.next_publish = self.now + self.publish_period
        if outcome!=TICKING:
            self.timer.cancel()            
            self.set_outcome(outcome)
            get_logger().info(f"BeTFSM finished with outcome {outcome}")
            self.shutdown_future.set_result(True)         

    def set_outcome(self, outcome):
        with self.outcome_lock:
            self.outcome = outcome

    def get_outcome(self):
        with self.outcome_lock:
            outcome = self.outcome
        return outcome

    def run(self):
        self.first_time = True
        rclpy.spin_until_future_complete(self.node, self.shutdown_future, executor=None,timeout_sec=None)
        

