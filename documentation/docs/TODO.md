# TODO list


- Investigate the difference between reset() and cancel_state().  How does the original yasmin code handles concurrent access?

- clean up the outcomes list

- uniform treatment of names

- generating graphviz representation of state machine.

- _outcomes and outcomes : list are duplicate, use only one, change everything to _outcomes to be compatible with State.
  (dynamically built outcomes know need a dirty fix self._outcomes = outcomes )



- uniform logging (problem:  should work with/without ROS2)

