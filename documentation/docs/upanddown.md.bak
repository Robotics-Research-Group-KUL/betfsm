# Tutorial example "Up & Down"

This introduction showcases several ways to define a state machine, and shows some  
of the possibilities of BeTFSM:

Different BeTFSM nodes are implemented in different ways: 

- Up & Down, specified as a function:   specifies a simple sequence of actions.
- Up & Down, specified as a function and with a timer running concurrently: concurrently starts a timer that goes off at 3 seconds,
  and executes a sequence of actions.  The whole tree returns SUCCEED when the timer goes passed 3 seconds.
- Up & Down, specified as a class: specifies a simple sequence using a class.  A class is usefull when you need
  to perform more advanced configuration or construction of the children of the sequence.
- Up & Down, with eTaSL tasks with input and output parameters: An example that shows how to pass parameters to and from eTaSL 
  state machines.
- Up & Down, with eTaSL tasks with input and output parameters using lambda functions: Often it is useful to use python lambda
  function to ensure execution at run-time.

The example library with state machines also shows the definition of a schema for the action server.
This schema (called my_schema) defines the parameters of the action.

### Full code of up-and-down examples:
<!--codeinclude-->
[full code of the up-and-down examples](../../betfsm_demos/betfsm_demos/sm_up_and_down.py)
<!--/codeinclude-->


