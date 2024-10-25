# Tutorial example "Up & Down"

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
   ros2 run yasmin_etasl_demos example_action_server
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
   ros2 run yasmin_etasl web_server
```
I you look at the action scripts, you see that a [Concurrent][yasmin_etasl.yasmin_ticking.Concurrent] state runs the state
machine in parallel with a GraphvizPublisher state that publishes the state machine on a topic `/gz`.  There is a 
`do_not_expand_types` and `do_not_expand_instances` argument where you can limit what is displayed in the state machine
(in order not to overload the figure)
```
Concurrent("concurrent",[
 sm,
 GraphvizPublisher("publisher","/gz",sm,None,skip=10,do_not_expand_types=[eTaSL_StateMachine])
```


!!! Warning "TODO"
    still some work here...

in `sm_up_and_down.py` you find example statemachines that show different styles for specifying
a simple state machine that moves up and down:

 
### Up & Down, specified as a function:   
:::yasmin_etasl_demos.sm_up_and_down.up_and_down_as_a_function
   options:
      heading_level: 3
      show_source: true
      show_root_heading: true   

### Up & Down, specified as a function and with a timer running concurrently:
:::yasmin_etasl_demos.sm_up_and_down.up_and_down_as_a_function_and_a_timer
   options:
      heading_level: 3
      show_source: true
      show_root_heading: true   

### Up & Down, specified as a class:
:::yasmin_etasl_demos.sm_up_and_down.Up_and_down_as_a_class
   options:
      heading_level: 3
      show_source: true
      show_root_heading: true   

### Up & Down, with eTaSL tasks with input and output parameters:
:::yasmin_etasl_demos.sm_up_and_down.Up_and_down_with_parameters
   options:
      heading_level: 3
      show_source: true
      show_root_heading: true   
      

### Up & Down, with eTaSL tasks with input and output parameters using lambda functions:
:::yasmin_etasl_demos.sm_up_and_down.Up_and_down_with_parameters_lambda
   options:
      heading_level: 3
      show_source: true
      show_root_heading: true   
