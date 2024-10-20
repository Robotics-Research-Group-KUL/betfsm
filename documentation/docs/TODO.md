# TODO list


- Investigate the difference between reset() and cancel_state().  How does the original yasmin code handles concurrent access?

- clean up the outcomes list

- uniform treatment of names

- generating graphviz representation of state machine.

- _outcomes and outcomes : list are duplicate, use only one, change everything to _outcomes to be compatible with State.
  (dynamically built outcomes know need a dirty fix self._outcomes = outcomes )



- uniform logging (problem:  should work with/without ROS2)




## Idea : Waiting + chaining

### Problem

A typical subtask is to wait for a condition while doing something.
- waiting until a condition on the blackboard
- waiting until end of task (received by event topic)
- waiting until a timeout
- reading topic and possibly until condition on topic message

### How do we model this:



- (Concurrent)Fallback (or *Any(success)*):   Implements a behaviortree-like Fallback node (concurrently executed):

    - success any other outcome,
    - failure is CANCEL outcome

  Finishes if **any** has success.  Success is defined by an outcome different from CANCEL.
  In other words, success can be differentiated by different outcomes.

- (Concurrent)Sequence (or *All(success)*). Implements a behaviortree-like Sequence node (concurrently executed):
    - success is SUCCEED outcome,
    - failure is any other outcome

  Finishes if **all** have success. Success i-s defined by an outcome SUCCEED.
  In other words, failure can be differentiated by different outcomes.


So, for an eTaSL state:

  ConcurrentFallback( EventTopic, OutputTopic, Sequence(Activate, Wait) )

  But for EventTopic this means that the topics have to be with a durability = transient local
  because the topic can be created late. Could we tune the qos parameters to be transient local for
  a small time, i.e. 0.1 (or even smaller) ?


relatively short Lifespan and transient local durability are probably the best.