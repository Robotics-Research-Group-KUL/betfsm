# Local references and get_path


The function [get_path][betfsm.get_path] knows about relative references such as ".", "./my_name>", "../my_name", etc...
The routine also has a TickingState as input, a unique identifier for this state is used
as an index in a blackboard[local] table.   In this way we can have blackboard location that is private
for a state and its children.  The ".." uses the state to lookup its parent and will store in the
parent's blackboard location.

Note that these semantics are related but different from a typical directory path: there is no direct way for a
parent to get to the local space of the children only using a path.   

Sometimes a parent class want to pass a callback (with a path parameter) to one of its children.  It would be confusing
for the user of this parent that he would need to know what happens inside this parent. There is ConfigCallback class
[CC_set_state][betfsm.CC_set_state] that take care of this, such that it seems for the user that he just gives the callback to the parent.


