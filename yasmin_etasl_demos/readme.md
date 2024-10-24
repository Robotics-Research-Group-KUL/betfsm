# yasmin_etasl_demos

Some demonstrations of the yasmin_etasl library.


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
  ```
The **eTaSL node** runs the eTaSL controller:
  ```
  ros2 run etasl_ros2 etasl_node
  ```

## Defining your state machine

The `yasmin_etasl_demos/sm_up_and_down.py` files definesexample state-machines, always very with a very similar robot tasks, but demonstrating different features and styles:
- Up & Down, specified as a function.
- Up & Down, specified as a function a.d with a timer running concurrently.
- Up & Down, specified as a class.
- Up & Down, with eTaSL tasks with input and output parameters.
- Up & Down, with eTaSL tasks with input and output parameters using lambda functions (**this is the one used by the action server**)


For a state machine example you can look at `eTaSL_StateMachine` in `yasmin_etasl.yasmin_ticking_etasl`.

## Running the action server example:

The example action server in this repository is run by:
  ```
  ros2 run yasmin_etasl_demos example_action_server
  ```
You are now listening for incomming actions.
A small script in the `./scripts` directory calls an action handled by the example_action_server:
  ```
  ./scripts/send_up_and_down.sh
  ```
Try interrupting the script with ctrl-C while its waiting to finish. You'll see that this is passed to
the state-machine, that will also finish, this is done by checking a variable on the blackboard
while running your statemachine:
```python
  While("While",lambda bm: not bm["cancel_goal"], <your statemachine>) 
```
Cleanup motions after canceling can be defined using a `Fallback` state (if the first fails, the second is tried and so one ...)


