# Ticking state machine

This page lists classes that are used to define appropriate behaviors for ["TickingStateMachines"][yasmin_etasl.tickingstatemachine.TickingStateMachine] :

  - State machines that can deal with TickingStates that return TICKING and resume at the appropriate place
    when recalled. Transitions in the state machine can come from the output of an underlying state or *from an external source*(Listener).
  - that can have an associated ["Listener"][yasmin_etasl.tickingstatemachine.Listener] which listens from an external source and injects them
    in the statemachine. 
    
    One example of such a ["Listener"][yasmin_etasl.tickingstatemachine.Listener] is an ["EventTopicListener"][yasmin_etasl.yasmin_ticking_ros.EventTopicListener] which listens to a ROS topic and uses its `process_message` method to derive an outcome from the topic message.  its `set_payload` method to possibly put some information from the topic message to the blackboard.
    
    [eTaSLEventTopicListener][yasmin_etasl.yasmin_ticking_etasl.eTaSLEventTopicListener] is a specialization of ["EventTopicListener"][yasmin_etasl.yasmin_ticking_ros.EventTopicListener] that knows how to process eTaSL's string event topics.
    
  - this ["Listener"][yasmin_etasl.tickingstatemachine.Listener]  has a queuing policy defined by [Queue][yasmin_etasl.tickingstatemachine.Queue]

[eTaSL_StateMachine][yasmin_etasl.yasmin_ticking_etasl.eTaSL_StateMachine] is a ["TickingStateMachines"][yasmin_etasl.tickingstatemachine.TickingStateMachine] that uses a 


::: yasmin_etasl.tickingstatemachine.Queue
    options:
      show_source: false
      show_root_heading: true 



::: yasmin_etasl.tickingstatemachine.QueueFIFO
    options:
      show_source: false
      show_root_heading: true 



::: yasmin_etasl.tickingstatemachine.QeueuFIFOfilter
    options:
      show_source: false
      show_root_heading: true 




::: yasmin_etasl.tickingstatemachine.Listener
    options:
      show_source: false
      show_root_heading: true 

::: yasmin_etasl.yasmin_ticking_ros.EventTopicListener
    options:
      show_source: false
      show_root_heading: true 

::: yasmin_etasl.yasmin_ticking_etasl.eTaSLEventTopicListener
    options:
      show_source: false
      show_root_heading: true

::: yasmin_etasl.tickingstatemachine.TickingStateMachine
    options:
      show_source: false
      show_root_heading: true 

::: yasmin_etasl.yasmin_ticking_etasl.eTaSL_StateMachine
    options:
      show_source: false
      show_root_heading: true 