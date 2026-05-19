# Managing the LifeCycle of a ROS2 node.

[LifeCycle][betfsm_ros.LifeCycle] manages the lifecycle of some other ROS2 node.
  The node is constructed with service_name, transition (see below), timeout and the transition is requested during execution of the state.
  See below for a simplified view of this lifecycle (transition states not included):
```mermaid
    stateDiagram-v2
        direction LR
        classDef successClass  fill:darkgreen,color:white
        classDef tickingClass  fill:yellow,color:black
        classDef otherClass  fill:darkorange,color:white
        classDef abortClass  fill:darkred,color:white

        
        [*] --> unconfigured
        unconfigured --> inactive : CONFIGURE
        inactive --> active : ACTIVATE
        inactive --> unconfigured : CLEANUP
        inactive --> finalized : INACTIVE_SHUTDOWN
        active --> inactive : DEACTIVATE
        active --> finalized : ACTIVE_SHUTDOWN
        unconfigured --> finalized : UNCONFIGURED_SHUTDOWN
```

   These are the transtions that can be passed to a LifeCycle state:
    ```python
        class Transition(Enum):
            CONFIGURE              = 1
            CLEANUP                = 2
            ACTIVATE               = 3
            DEACTIVATE             = 4
            UNCONFIGURED_SHUTDOWN  = 5
            INACTIVE_SHUTDOWN      = 6
            ACTIVE_SHUTDOWN        = 7    
    ```
