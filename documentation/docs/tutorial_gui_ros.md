## Executing a BeTFSM Tree under ROS2 with cROSpi

!!! warning
    This Section and the later tutorials have to be elaborated


### CrospiNode

### TopicEvents and interruptions

### CrospiOutput


### Parameters


### CrospiInput

- different ways of running


Besides using the **ROSRunner** class, there is some ROS2 boiler-plate needed to 
initialize **rclpy** ROS2 interface, start-up and destroy the ROS2 node, and to shut **rclply** down afterwards.
**load_task_list** loads a list of task specifications, specified in a configuration file.  The **eTaSL_StateMachine**
node communicates with cROSpi to execute the task with the given name:

```python linenums="1"
--8<-- "betfsm_demos/betfsm_demos/skill_example_2.py:29"
```


### Handling interruptions

In a real-life robot execution, one needs to be careful to handle all exceptional situations
such that the robot system always ends up in a predictable state.



#### Directly interrupting and cleaning-up

A first solution is to directly interrupting the running state machines
and *cleaning up* afterwards by calling the appropriate service calls
to stop the robot.

```python linenums="1"
--8<-- "betfsm_demos/betfsm_demos/skill_example_3.py:29"
```

#### Directly interrupting and cleaning-up

A second solution is to register the interruption and only
react to it at a given state in the state machine.  This would allow e.g.
a robot to continue until he is in a safe state before stopping.

```python linenums="1"
--8<-- "betfsm_demos/betfsm_demos/skill_example_3.py:29"
```




