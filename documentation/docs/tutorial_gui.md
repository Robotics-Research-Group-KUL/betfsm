## Graphical user interface for BeTFSM

### Running the BeTFSM web-server user interface


BeTFSMRunnerGUI is not only a graphical user interface to monitor a BeTFSM-tree, but is also a way to configure
your BeTFSM from the command prompt.

Using the graphical user interface is very easy.  One needs just to replace
the BeTFSMRunner with [BeTFSMRunnerGUI][betfsm.betfsmrunnergui.BeTFSMRunnerGUI].  A webserver will be started that serves
the graphical user interface at `localhost:8000`.


``` py
sm = build_tree()
runner = BeTFSMRunnerGUI(sm,bb, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
outcome=runner.run()
```


A full example can be found in ```betfsm_examples/guiexample_simple1.py```
```
import threading
import webbrowser

# ---- building the state machine -----
from betfsm.betfsm import (
        Sequence, ConcurrentSequence, TimedWait, TimedRepeat, Message, SUCCEED, Generator, Repeat
)
from betfsm.logger import get_logger
from betfsm.betfsmrunnergui import BeTFSMRunnerGUI

# ---------------------------------------
def build_tree():
    sm = Sequence("concurrent_sequence_outer", [
        Message(msg="This demo uses ConcurrentSequence, Sequence, TimedRepeat"),
        Message(msg="--- concurrent_sequence started ---"),
        ConcurrentSequence("concurrent_sequence", [
            Sequence("sequence1", [
                Message(msg="   --- sequence 1 started ---"),
                TimedRepeat("timedrepeat1", 5, 5, Message(msg="      sequence 1: 5 times every 5 second")),
                Message(msg="   --- sequence 1 ended   ---")]),
            Sequence("sequence2", [
                Message(msg="   --- sequence 2 started ---"),
                TimedRepeat("timedrepeat1", 10, 6, Message(msg="      sequence 2: 10 times every 6 second")),
                Message(msg="   --- sequence 2 ended   ---")]),
            Sequence("sequence3", [
                TimedWait("waiting 20 sec", 20.0),
                Message(msg="I like to interrupt!", logFunc=get_logger().warn)]),
        ]),
        Message(msg="--- concurrent_sequence ended   ---")
    ])
    return sm


# ---------------------------------------

def main():
    bb = {}
    sm = build_tree()
    runner = BeTFSMRunnerGUI(sm,bb, frequency=100.0, publish_frequency=5.0, debug=False, display_active=False)
    runner.run()

if __name__ == "__main__":
    main()

```

Other examples are provided in ```guiexample_simple2.py``` and ```guiexample_large.py```.

### Using the GUI

Go with a webbrowser to `localhost:8000` after you have started the BeTFSM server.  Once you loaded the page, you can start/stop the server and the page will automatically reload if the server becomes available again (checking every 5 seconds).

The GUI displays a tree representing the BeTFSM tree. The orange nodes are currently active.

In the GUI nodes can be expanded or collapsed by pressing the "-" or "+" ons the right side of each node.  You can also expand all nodes or collapse all nodes.

Very handy is the AutoExpand option, this will collapse the tree but expand all nodes that are needed to visualize all **active** nodes.

The Play button and sliding bar are currently not implemented. Their purpose will be to go back and forward in time using a history of the execution.

![GUI](static/gui.png)

### BeTFSMRunnerGUI is more than a GUI

This runner also offers configurability on the command line:

```
usage: your_betfsm_prog.py [-h] [--frequency FREQUENCY]
                           [--publish_frequency PUBLISH_FREQUENCY] [--debug | --no-debug]
                           [--display_active | --no-display_active]
                           [--betfsm_log BETFSM_LOG] [--generate_dot GENERATE_DOT]
                           [--generate_json GENERATE_JSON] [--serve | --no-serve]
                           [--host HOST] [--port PORT] [--workers WORKERS]
                           [--log-level {critical,error,warning,info,debug,trace}]

BeTFSMRunnerGUI command line options

options:
  -h, --help            show this help message and exit

BeTFSMRunnerGUI Options:
  --frequency FREQUENCY
                        frequency at which BeTFSM runs [default:100.0]
  --publish_frequency PUBLISH_FREQUENCY
                        publishing frequency for GUI [default:5.0 ]
  --debug, --no-debug   Log the timing of each tick [default: False]
  --display_active, --no-display_active
                        Log the active nodes at rate equal to publish_frequency[default:
                        False]
  --betfsm_log BETFSM_LOG
                        BeTFSM Log specification string, a comma-separated list of
                        category:level e.g. 'default:INFO, state:FATAL' with levels
                        DEBUG,INFO,WARNING,ERROR,FATAL. Known categories are default and
                        state, but there can be user-defined categories [default: '']
  --generate_dot GENERATE_DOT
                        generate a graphviz .dot file from the state machine and store in
                        the specified file (and quit the program without running)
  --generate_json GENERATE_JSON
                        generate a json file from the state machine and store in the
                        specified file (and quit the program without running)
  --serve, --no-serve   Start-up server with graphical user interface [default:True]

Uvicorn Web Server Options:
  --host HOST           Bind socket to this host [default: 0.0.0.0]
  --port PORT           Bind socket to this port [default: 8000]
  --workers WORKERS     Number of worker processes[default: 1]
  --log-level {critical,error,warning,info,debug,trace}
                        log-level of the web-server [default: info ]
```


- It allows you to set the parameters of the web-server, e.g. to configure it to run from a docker image. 
- It also can configure the BeTFSM logging and the web-server logging.
- It allows you to **generate graphviz dot** files and json files describing the BeTFSM tree. These can
be visualized using GraphViz ("sudo apt install xdot"    and "xdot filename.dot")
- You can turn-off webserver and GUI if you only want the configurability.

See the constructor of [BeTFSMRunnerGUI][betfsm.betfsmrunnergui.BeTFSMRunnerGUI] on how to set the parameters
of the BeTFSMRunnerGUI, these parameters can be overridden by the command-line parameters.  

