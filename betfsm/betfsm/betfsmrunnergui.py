import uvicorn
import threading
import time
from betfsm.backend.app import app,publish_tick,set_state_machine
from betfsm.betfsm import TickingState,Blackboard,TICKING

class BeTFSMRunnerGUI:
    """
    Runner for a BeTFSM ticking state machine.
    Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
    runs in the main thread.  You typically call this class in the main body of your program.
    """
    def __init__(self, statemachine: TickingState, blackboard: Blackboard, frequency: float=100.0, publish_frequency=10,debug: bool = False, display_active=False, serve=True):
        """
        Initializes the BeTFSMRunner.  This BeTFSMRunner has no other dependencies and
        runs in the main thread.

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
        """
        self.statemachine = statemachine
        self.blackboard = blackboard
        self.frequency = frequency
        self.interval_sec = 1.0/frequency
        self.debug = debug
        self.display_active = display_active
        self.publish_period = 1.0/publish_frequency
        set_state_machine(statemachine)  # set state machine for web-app

        self.serve = serve
        if self.serve:
            set_state_machine(statemachine)
            threading.Thread(
                target=lambda: uvicorn.run(app, host="0.0.0.0", port=8000, reload=False),
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


