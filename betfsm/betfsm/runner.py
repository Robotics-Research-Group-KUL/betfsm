# betfsmrunnergui.py 
#
# Copyright (C) Erwin Aertbeliën, 2025
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


import json
import sys
import math
import time

import uvicorn
import threading
import argparse
import time

from betfsm import (
    TickingState,Blackboard,TICKING,get_logger, set_loggers_from_specification_string, 
    get_logger_categories,LogPrinter
)
from .statemachine_visitor import StateMachineVisitor
from .jsonvisitor import JsonVisitor
from .graphviz_visitor import to_graphviz_dotfile
from betfsm.backend.app import app,publish_tick,set_webserver_param


class TimerStats:
    def __init__(self):
        self.reset_all()
    
    def reset(self):
        self.sum   = 0
        self.N     = 0
        self.sumsq = 0
        self.min   = math.inf
        self.max   = -math.inf
    
    def reset_all(self):
        self.reset();
        self.global_N = 0
        self.global_sum   = 0
        self.global_sumsq = 0
        self.global_min   = math.inf
        self.global_max   = -math.inf        

    def add(self, value):
        sq = value*value
        self.N     += 1
        self.sum   += value        
        self.sumsq += sq
        self.min   = min(self.min, value)
        self.max   = max(self.max, value)
        self.global_N     += 1
        self.global_sum   += value        
        self.global_sumsq += sq
        self.global_min   = min(self.global_min, value)
        self.global_max   = max(self.global_max, value)        

    def cycle(self):
        if self.N==0:
            return f"N={0:5d}"
        mean = self.sum / self.N
        var  = self.sumsq / self.N - mean*mean
        std  =  math.sqrt(var)
        return f"N={self.N:5d} mean = {mean:10.6f}, std = {std:10.6f}, min = {self.min:10.6f}, max = {self.max:10.6f}"

    def all(self):
        mean = self.global_sum / self.global_N
        var  = self.global_sumsq / self.global_N - mean*mean
        std  =  math.sqrt(var)
        return f"N={self.N:5d} mean = {mean:10.6f}, std = {std:10.6f}, min = {self.global_min:10.6f}, max = {self.global_max:10.6f}"


class RunnerBase:

    # defined as separate function such that you can inject additional arguments if needed.
    # override this in a subclass and call the super() of this function and further adapt.
    # will be called in __init__(...), 

    def parse_arguments(self,parser,group_app,group_uvr):
        """ 
        Parses the arguments from the command-line.  This function
        could further add parameters to the parser if necessary.
        """
        assert("should be implemented by subclas")
    
    # __init__() ensures that parsed arguments are available at self.args
    #
    def __init__(self, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency : float=10,debug: bool = False, 
                 display_active:bool =False, betfsm_log:str=None, name_filter:str="", type_filter:str="",  
                 allow_generate_dot:bool=True, allow_generate_sm_dot:bool=True,
                 allow_generate_json:bool=True,serve:bool=True,
                 host:str="0.0.0.0", port:int=8000, workers:int=1, log_level:str="info"):
        """
        Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
        runs in the main thread.  The parameters frequency, publish_frequency, debug, display_active,
        and serve can be overridden by command-line parameters.  Use "--help" to show all command-line
        parameters.

        Parameters:
            statemachine:
                the TickingStateMachine to be run

            blackboard(Blackboard):
                the blackboard to be used
            frequency:
                frequency at which the statemachine is ticked, in Hz [default=100Hz]
            publish_frequency:
                frequency at which to publish to the webbrowser [default= 10Hz]
            debug:
                If true outputs debug info on console each tick. [default=False]
            display_active:
                displays all active nodes on log at publish_frequency. Only if activity changed!
            betfsm_log:
                A list of categories, separated by ':' that will be logged, known categories
                are default, state, service,... But users and libraries can add their own.
            name_filter:
                comma-separated list of names of nodes that are not descended into in
                the graphical user interface
            type_filter:
                comma-separated list of typrs of nodes that are not descended into in
                the graphical user interface                
            allow_generate_dot:
                adds the generate_dot command-line parameter. [default: True]
            allow_generate_sm_dot:
                adds the generate_sm_dot command-line parameter. [default: True]                
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
            log_level:
                log-level of the web-server (not BeTFSM), default = "info"
                choices are "critical", "error", "warning", "info", "debug", "trace"

        """
        self.statemachine = statemachine
        self.blackboard   = blackboard      

        epilog="""
        You can still pass ROS2 arguments according to ROS2 conventions: e.g.
        adding : "--ros-args --log-level debug" to your arguments
        """        
        # define command line arguments in two groups: application and webserver (unvicorn)
        parser = argparse.ArgumentParser(description="BeTFSMRunnerGUI command line options",epilog=epilog)
        
        group_app = parser.add_argument_group("BeTFSMRunnerGUI Options")
        group_app.add_argument("--frequency",type=float, default=frequency, help=f"frequency at which BeTFSM runs [default:{frequency}]")
        group_app.add_argument("--publish_frequency",type=float, default=publish_frequency, help=f"publishing frequency for GUI [default:{publish_frequency} ]")
        group_app.add_argument("--debug", action=argparse.BooleanOptionalAction, default=debug,help=f"Log statistics of the timing of each tick at publish_frequency [default: {debug}]")
        group_app.add_argument("--display_active",action=argparse.BooleanOptionalAction,default=display_active, help=f"Log the active nodes at rate equal to publish_frequency, only logs changes to activity[default: {display_active}] ")
        group_app.add_argument("--betfsm_log",type=str,default=betfsm_log, help=f"BeTFSM Log specification string, i.e. a colon separated list of categories. Known categories are {get_logger_categories()} but there can be user-defined categories   [default: '{betfsm_log}'] ")
        group_app.add_argument("--name-filter",type=str,default=name_filter, help=f"specifies a colon-separated list of names (can be regular expressions) to filter out.[default: '{name_filter}'")
        group_app.add_argument("--type-filter",type=str,default=name_filter, help=f"specifies a colon-separated list of types not to descent into.[default: '{type_filter}'")
        if allow_generate_dot:
            group_app.add_argument("--generate_dot",type=str, default="", help="generate a graphviz .dot file from the state machine and store in the specified file (and quit the program without running)")
        if allow_generate_dot:
            group_app.add_argument("--generate_sm_dot",type=str, default="", help="generate a graphviz .dot file for the state machine named in --name-filter, and stored it in the given file (and quit program without running)")
        if allow_generate_json:
            group_app.add_argument("--generate_json",type=str, default="", help="generate a json file from the state machine and store in the specified file (and quit the program without running)")
 
        group_app.add_argument("--serve",action=argparse.BooleanOptionalAction,default=serve,help=f"Start-up server with graphical user interface [default:{serve}]")
        # We use the exact names uvicorn.run() expects as kwargs
        group_uvr = parser.add_argument_group("Uvicorn Web Server Options")
        group_uvr.add_argument("--host", type=str, default=host, help=f"Bind socket to this host [default: {host}]")
        group_uvr.add_argument("--port", type=int, default=port, help=f"Bind socket to this port [default: {port}]")
        group_uvr.add_argument("--workers", type=int, default=workers, help=f"Number of worker processes[default: {workers}]")
        group_uvr.add_argument("--log-level", type=str, default=log_level, 
                               choices=['critical', 'error', 'warning', 'info', 'debug', 'trace'], 
                               help=f"log-level of the web-server [default: {log_level} ]")
        
        

        # get parser arguments 
        args              = self.parse_arguments(parser,group_app, group_uvr)       
        
        get_logger().info(f"BeTFSMRunnerGUI parameters: {args}")        

        # store arguments in class:
        self.args = args        
        self.serve          = args.serve
        self.debug          = args.debug
        self.display_active = args.display_active                

        # handle commands to generate files:
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
                print(f"json file '{args.generate_json}' is generated, and program will be terminated")                    
                sys.exit()
        if allow_generate_sm_dot:
            if args.generate_sm_dot:
                if args.name_filter=="":
                    print("when using --generate_sm_dot you should also specify a single name with --name-filter")                    
                    sys.exit(1)
                if ":" in args.name_filter:
                    print("when using with --generate_sm_dot you should only specify a single name using --name-filter")                    
                    sys.exit(1)                    
                visitor = StateMachineVisitor(args.name_filter)
                statemachine.accept(visitor)
                with open(args.generate_sm_dot,"w") as f:
                    f.write( visitor.result() )
                print(f"graphviz file '{args.generate_sm_dot}' for state machine {args.name_filter} is generated, and program will be terminated")
                sys.exit()
        # call uvicorn.run()        
        uvicorn_kwargs = {
            "host": args.host,
            "port": args.port,
            "workers": args.workers,
            "log_level": args.log_level,
        }
        if self.serve:
            self.name_filter = args.name_filter
            self.type_filter = args.type_filter
            set_webserver_param(statemachine,self.blackboard,self.name_filter,self.type_filter)  # set parameters for web-app
            threading.Thread(
                target=lambda: uvicorn.run(app, **uvicorn_kwargs),
                daemon=True
            ).start()        
        self.stats = TimerStats()

        
                
    def initialize(self):
        if self.serve or self.display_active:
            TickingState.global_publish_log = {}  # turn on global_publish_log
        self.set_outcome("TICKING")       
        self.previous_active = []
        self.stats.reset_all()

    def process_publish_cycle(self):
        # ensure the last one is always published
        if self.serve:
            publish_tick() # publishes all states that where active between calls to publish_tick
        if self.display_active:
            if TickingState.global_publish_log != self.previous_active:
                active = "("
                for k,v in TickingState.global_publish_log.items():
                    active = active + v.name + " "
                active=active+")"
                get_logger().info(f"ACTIVE : {active}")
                self.previous_active = TickingState.global_publish_log.copy()
        if TickingState.global_publish_log is not None:
            TickingState.global_publish_log.clear()               
        if self.debug:
            print("debug")
            get_logger().info(f"Timer stat. : {self.stats.cycle()}")
            self.stats.reset()

    def finalize(self):
        get_logger().info(f"Runner finished with outcome {self.get_outcome()}")
        get_logger().info(f"Global timer stat. : {self.stats.all()}")

    def set_outcome(self, outcome):
        self.outcome = outcome

    def get_outcome(self):
        return self.outcome
    
    def run(self):
        assert("RunnerBase.run() should have been overriden by a subclass ")


        

class Runner(RunnerBase):
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.

    This is multi-threaded, only access member variables using the methods designed for this. (i.e. set_outcome, get_outcome)
    """    
    def __init__(self, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency=5,debug: bool = False, 
                 display_active=False, betfsm_log=None,logger=LogPrinter(),name_filter:str="",type_filter:str="", 
                 allow_generate_dot=True, allow_generate_sm_dot:bool=True,allow_generate_json=True,serve=True,
                 host="0.0.0.0", port=8000, workers=1, log_level="info"):
   
        super().__init__(statemachine, blackboard, frequency, publish_frequency, debug, display_active, betfsm_log,name_filter,type_filter,
                         allow_generate_dot,allow_generate_sm_dot,allow_generate_json,serve,host, port, workers, log_level)
        args = self.args

        # ROS dependend: everything timer related, logger, node, clocks, synchronization primitives:
        self.frequency       = args.frequency
        self.interval_sec    = 1.0/args.frequency
        self.publish_period  = 1.0/args.publish_frequency  
        set_loggers_from_specification_string(args.betfsm_log, logger)

    def parse_arguments(self, parser, group_app, group_uvr):
        clean_args        = sys.argv[1:]
        args              = parser.parse_args(clean_args)        
        return args
    
    def run(self) -> int:
        """
        Runs the statemachine until it returns an outcome different from TICKING

        Refers to absolute time to avoid drifting. Uses monotonic clock to 
        avoid problems with system time changes.

        Returns:
            the final outcome of the statemachine
        """
        # Use monotonic clock to avoid issues with system time changes
        # start    = time.monotonic()
        # self.initialize()
        # next_run = start + self.interval_sec        
        # now      = start
        # if self.debug:
        #     get_logger().debug(f"Runner: time: {start:10.4f} s started (frequency:{self.frequency})")
        
        # next_publish = now
        outcome = TICKING


        now          = time.monotonic()
        previous_run = now - self.interval_sec           
        next_publish = now                                   # ensure publishing at start
        self.initialize()
        if self.debug:
            get_logger().info(f"ROSRunner: started at {now/1.0E9:10.4f} s started (frequency:{self.frequency})")        
        while True:
            outcome = self.statemachine(self.blackboard)            
            jitter = (now - previous_run) - self.interval_sec
            if abs(jitter) > self.interval_sec*0.5:   
                get_logger().warn(f"Timing: large deviation : {jitter} s")   
            self.stats.add(jitter)                  
            if (now >= next_publish - self.interval_sec*0.5) or (outcome != TICKING):
                self.process_publish_cycle()
                next_publish = now + self.publish_period                            
            previous_run = now            
            # Sleep until the next scheduled time
            sleep_time = previous_run + self.interval_sec - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, log drift
                if abs(sleep_time) > 0.5*self.interval_sec:
                    get_logger().warn(f"Timing: large deviation {-sleep_time:.6f} s late")                
            now          = time.monotonic()                                                     
            if outcome!=TICKING:                
                break
        # while outcome == TICKING:
        #     outcome = self.statemachine(self.blackboard)
        #     if (now >= next_publish - self.interval_sec/2) or (outcome != TICKING):
        #         self.process_publish_cycle()
        #         next_publish = now + self.publish_period
        #     now = time.monotonic()
        #     # Sleep until the next scheduled time
        #     sleep_time = next_run - now
        #     if sleep_time > 0:
        #         time.sleep(sleep_time)
        #     else:
        #         # If we're behind schedule, log drift
        #         get_logger().warn(f"[Warning] Drift detected: {abs(sleep_time):.3f} s late")
        #     # Schedule next run
        #     if self.debug:
        #         self.stats.add((now - next_run) )
        #     next_run += self.interval_sec
        # get_logger().info(f"Runner finished with outcome {outcome}")            
        # self.finalize()
        self.set_outcome(outcome)
        self.finalize()
        return outcome




