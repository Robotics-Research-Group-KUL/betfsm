# betfsm_demos 

Some demonstrations of the BeTFSM library, together with the action server and eTaSL.

A behavior tree and finite state machine framework in Python using cooperative multitasking where new state machines or behavior-tree nodes can be easily specified.
It provides:

- Hierarchical (composite) states
- Parametrizable and reusable states
- Easy to build statemachines by composing existing states/state machines using Behavior-tree principles.
- Easily extendable using Python's generators and yield.
- On-line visualization of the tree-structure.


## Setup up your environment
If you are running from a devcontainer or docker container, 
one needs to give permission for the devcontainer to display graphical windows on the host computer.
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
## Setup up eTasL with RVIZ
**RVIZ** : visualization of UR10 robot:
  ```
  ros2 launch etasl_ros2_application_template load_ur10_setup.launch.py
  ``` The **eTaSL node** runs the eTaSL controller: ```
  ros2 run etasl_ros2 etasl_node
  ```

## Defining your state machine

The `betfsm_demos/sm_up_and_down.py` files definesexample state-machines, always very with a very similar robot tasks, but demonstrating different features and styles:
- Up & Down, specified as a function.
- Up & Down, specified as a function a.d with a timer running concurrently.
- Up & Down, specified as a class.
- Up & Down, with eTaSL tasks with input and output parameters.
- Up & Down, with eTaSL tasks with input and output parameters using lambda functions (**this is the one used by the action server**)


For a state machine example you can look at `eTaSL_StateMachine` in `betfsm.betfsm_etasl`.

## Running the action server example:

Running the eTaSL node:
```
ros2 run etasl_ros2 etasl_node
```
This will start up a server that can execute eTaSL.

The example action server in this repository is run by:
  ```
  ros2 run betfsm_demos example_action_server
  ```
You are now listening for incomming actions.
A small script in the `./betfsm_demos/scripts` directory calls an action handled by the example_action_server:
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



