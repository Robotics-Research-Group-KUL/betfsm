# TODO list


- Investigate the difference between reset() and cancel_state().  How does the original yasmin code handles concurrent access?

- clean up the outcomes list

- uniform treatment of names

- generating graphviz representation of state machine.



## How do we model this:


So, for an eTaSL state:

  ConcurrentFallback( EventTopic, OutputTopic, Sequence(Activate, Wait) )

  But for EventTopic this means that the topics have to be with a durability = transient local
  because the topic can be created late. Could we tune the qos parameters to be transient local for
  a small time, i.e. 0.1 (or even smaller) ?


relatively short Lifespan and transient local durability are probably the best.