# Examples of Action Server

In the [Action server chapter](actionserver.md) you find the explanation of our Action Server framework.  This allows
BeTFSM to act as a ROS2 action server with a minimal effort from the user.

The following examples deal with a full example of eTaSL and a BeTFSM state machine,
together with an ActionServer example.

## Preparation / set-up when using a (simulated) robot

One needs to give permission for the devcontainer to display graphical windows on the host computer.
These needs to be executed on the host computer, outside Visual Studio.
  ```
  xhost +
  ```
To be checked if it also works (where local=localhost and root is users)
  ```
  xhost +local:root
  ```
To retract the permissions:
  ```
  xhost -
  ```

This test setup needs the following nodes running: 

**RVIZ** : visualization of UR10 robot:
```
  ros2 launch etasl_ros2_application_template load_ur10_setup.launch.py
```
Running the **eTaSL node**:
```
   ros2 run etasl_ros2 etasl_node
```
  This will start up a server that can execute eTaSL.

The **example action server** in this repository is run by:
```
   ros2 run betfsm_demos example_action_server
```

You are now listening for incomming actions.
A small script in the `./scripts` directory calls an action handled by the example_action_server:
```
   ./scripts/send_up_and_down.sh
```
Try interrupting the script with ctrl-C while its waiting to finish. You'll see that this is passed to
the state-machine, that will also finish, this is done by checking a variable on the blackboard
while running your statemachine.
There are two examples that deal in a different way with cancelation of the action:

 - `example_action_server.py` continuously checks for cancelation and interupts the robot motion by
   deactivating the eTaSL server.  The example shows how to specify a shutdown procedure.
 - `example_action_server2.py` uses a state to detect cancelation such that robot motion can be interrupted at a specified time.

These examples also start a state that publishes the statemachine on a String topic in graphviz format.

You can startup a webserver that publishes an svg file using:
```
   ros2 run betfsm web_server
```
I you look at the action scripts, you see that a [Concurrent][betfsm.betfsm.Concurrent] state runs the state
machine in parallel with a GraphvizPublisher state that publishes the state machine on a topic `/gz`.  There is a 
`do_not_expand_types` and `do_not_expand_instances` argument where you can limit what is displayed in the state machine
(in order not to overload the figure)
```
Concurrent("concurrent",[
 sm,
 GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
```




## example_action_server

### Description 
The `example_action_server` example shows a case where we continuously check for an action and when an
cancelation is detected go out of the state machines and perform a shutdown procedure.  The following class is
used for this purpose:

::: betfsm.betfsm_action_server.WhileNotCanceled
    options:
      heading_level: 4
      show_source: false
      show_root_heading: true 


This class runs its underlying state and has transitions that depend on the outcome of the underlying state or on cancelation requests.  With this we can build a simple statemachine to implement what should happen when cancel is requested:

```
class CheckingCancelAndShutdown(TickingStateMachine):
    def __init__(self,name:str,state:TickingState,srv_name:str="/etasl_node",timeout:Duration = Duration(seconds=1.0), node : Node = None):
        # execute in sequence but don't care about ABORT, only way to fail is TIMEOUT
        super().__init__(name,[CANCEL,SUCCEED])

        self.add_state(state=WhileNotCanceled("while_not_canceled",state), transitions={CANCEL:"DEACTIVATE_ETASL",SUCCEED:SUCCEED, TIMEOUT:"DEACTIVATE_ETASL"})

        self.add_state(state=LifeCycle("DEACTIVATE_ETASL",srv_name,Transition.DEACTIVATE,timeout,node),
                       transitions={SUCCEED: "CLEANUP_ETASL",  ABORT: "CLEANUP_ETASL", TIMEOUT:"CLEANUP_ETASL"} )
        self.add_state( state=LifeCycle("CLEANUP_ETASL",srv_name,Transition.CLEANUP,timeout,node),
                       transitions={SUCCEED: CANCEL, ABORT: CANCEL,TIMEOUT: CANCEL} )
```


### Full code
<!--codeinclude-->
[First example of action server:](../../betfsm_demos/betfsm_demos/example_action_server.py)
<!--/codeinclude-->


## example_action_server2


### Description 

The `example_action_server2` describes another approach where there is only a check for cancelation
at specific locations of the state machine(s).  The following TickingState is used for this purpose:


::: betfsm.betfsm_action_server.CheckForCanceledAction
    options:
      heading_level: 4
      show_source: false
      show_root_heading: true 


### Full code
<!--codeinclude-->
[Second example of action server:](../../betfsm_demos/betfsm_demos/example_action_server2.py)
<!--/codeinclude-->


