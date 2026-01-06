import uvicorn
import threading
import argparse
import time
from betfsm.backend.app import app,publish_tick,set_state_machine
from betfsm.betfsm import TickingState,Blackboard,TICKING
from betfsm.logger import get_logger

class BeTFSMRunnerGUI:
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.
    """
    def __init__(self, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency=10,debug: bool = False, display_active=False, serve=True,
                 host="0.0.0.0", port=8000, workers=1, log_level="info"):
        """
        Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
        runs in the main thread.  The parameters frequency, publish_frequency, debug, display_active,
        and serve can be overridden by command-line parameters.  Use "--help" to show all command-line
        parameters.

        Parameters:
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
                displays all active nodes on console
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

        parser = argparse.ArgumentParser(description="BeTFSMRunnerGUI command line options")

        # --- 1. YOUR Custom Arguments ---
        group_app = parser.add_argument_group("BeTFSMRunnerGUI Options")
        group_app.add_argument("--frequency",type=float, default=frequency, help=f"frequency at which BeTFSM runs [default:{frequency}]")
        group_app.add_argument("--publish_frequency",type=float, default=publish_frequency, help=f"publishing frequency for GUI [default:{publish_frequency} ]")
        group_app.add_argument("--debug", action=argparse.BooleanOptionalAction, default=debug,help=f"Log the timing of each tick [default: {debug}]")
        group_app.add_argument("--display_active",action=argparse.BooleanOptionalAction,default=display_active, help=f"Log the active nodes at rate equal to publish_frequency[default: {display_active}] ")
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
        args = parser.parse_args()


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
        self.debug = args.debug
        self.display_active = args.display_active
        self.publish_period = 1.0/args.publish_frequency
        set_state_machine(statemachine)  # set state machine for web-app
        self.serve = args.serve

        get_logger().info(f"BeTFSMRunnerGUI parameters: {args}")
        if self.serve:
            set_state_machine(statemachine)
            threading.Thread(
                target=lambda: uvicorn.run(app, **uvicorn_kwargs),
                daemon=True
            ).start()

    def run(self):
        """
        Runs the statemachine until it returns an outcome different from TICKING

        Refers to absolute time to avoid drifting. Uses monotonic clock to 
        avoid problems with system time changes.

        Returns:
            the final outcome of the statemachine
        """
        # Use monotonic clock to avoid issues with system time changes
        start    = time.monotonic()
        if self.debug:
            get_logger().debug(f"BeTFSMRunner time: {start:10.3f} s started (frequency:{self.frequency})")
        next_run = start + self.interval_sec
        outcome  = TICKING
        now      = start
        next_publish = now - self.publish_period
        TickingState.global_publish_log = {}  # turn on global_publish_log
        while outcome == TICKING:
            outcome = self.statemachine(self.blackboard)
            if now >= next_publish:
                publish_tick() # publishes all states that where active between calls to publish_tick!
                TickingState.global_publish_log.clear()
                next_publish = now + self.publish_period  
            now = time.monotonic()
            if self.display_active:
                gl=TickingState.get_global_log()
                active = "active: ("
                for k,v in TickingState.get_global_log().items():
                    active = active + v.name + " "
                active=active+")"
            else:
                active=""
            if self.debug or self.display_active:
                get_logger().debug(f"{now:10.3f} s : looping {active}")
            # Sleep until the next scheduled time
            sleep_time = next_run - now

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind schedule, log drift
                get_logger().warn(f"[Warning] Drift detected: {abs(sleep_time):.3f} s late")

            # Schedule next run
            next_run += self.interval_sec
        return outcome

