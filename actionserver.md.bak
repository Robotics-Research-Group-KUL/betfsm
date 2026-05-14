# Action Server

## Interface

The following class implements an action server that respond to actions
with idl:
```
# only from the perspective of the state-machine
# we can have generic handling of the parameters.
# (perhaps needing adaptation of your handling of eTaSL parameters, such that they can also take parameters that are set beforehand
# in the blackboard, i.e. not overwritten by defaults if they exist)
#
# goal
string task
string parameters
---
# result
string outcome
string parameters
---
# feedback
string state
string parameters
```

This interface is generic and applies to all state machines that are controlled:

- **goal**
  `task` is a label that indicates which state-machine needs to be executed
  `parameters` is a string with JSON describing the parameters of the task.  These parameters will be put in BeTFSM's blackboard for use within the statemachine.
  Optionally these parameters can be validated using a JSON-Schema 

- **result**
`outcome` is `cancel` or `success`
`parameters` is everything what is in `blackboard["result"]` encoded into a JSON string.

- **feedback**
`state` is the label of the currently active state of state-machine.
`parameters` is everything what is in `blackboard["feedback"]` encoded into a JSON string


## Server

![state machine of action server](./static/goal_state_machine.png)

The state machine is run in the EXECUTING state of the above figure from the ROS2 documentation.  The [goal_callback][betfsm.betfsm_action_server.BeTFSMActionServer.goal_callback] checks whether the task exists in the dictionary of state machines and optionally validates the input parameters of the task using a schema.  
The action server is implemented in the [BeTFSMActionServer][betfsm.betfsm_action_server.BeTFSMActionServer] described below.

The tasks that the action server can react to are described by a Dictionary that maps the name of the task to an instance of TickingState (a state machine)..


- Sequence diagram of interactions:
```mermaid
sequenceDiagram
  participant ActionClient
  participant ActionServer
  participant Execute
    
  activate ActionClient
  ActionClient ->> ActionServer: goal_request
  activate ActionServer
  ActionServer --) ActionClient: goal_response (accept only 1 simult.action)
  ActionServer ->> Execute: handle_goal
  deactivate ActionServer
  
  activate Execute
  ActionClient ->> ActionServer: get_result
  activate ActionServer

  
  
  Execute ->> Blackboard: set goal_handle
  Execute ->> Blackboard: set and validate input_parameters

  Note over Execute: state machine for task running
  
  
  
    
  loop over state machine until outcome!=TICKING
    Execute ->> Blackboard : get info from blackboard<br>always react at some point in time <br>to goal_handle.is_cancel_request<br>Generate feedback messages
  end
  Note over Execute: state machine stopped
  Execute ->> Blackboard : get result.outcome and <br> result.parameters from blackboard
  Execute ->> ActionServer: succeed() or cancel() <br> construct result msg

  
  deactivate Execute

  ActionServer--) ActionClient: result_request
   
  
  
  deactivate ActionServer
  deactivate ActionClient

```


To specify a schema to validate the parameters, add `input_parameters_schema` member to the TickingState with the schena in python format (i.e. using json.loads("..." )

::: betfsm.betfsm_action_server.BeTFSMActionServer
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 

## Giving intermediate feedback

::: betfsm.betfsm_action_server.FeedbackState
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 


## Checking for cancelation

Actions can be canceled and our action server and state machines need to react appropriately.

## Examples 

See [Action Server examples](actionserver_ex.md)

