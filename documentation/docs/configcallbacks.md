# Configuration callbacks : ConfigCallback

The BeTFSM tree is constructed in the beginning of the your application and typically remains static.  In order to pass parameters
during execution-time, callbacks are used.  [betfsm.ConfigCallback][betfsm.ConfigCallback] is an hierarchy of classes that attempts
to make the definition and use of these callbacks more systematic.

These are classes and not callback functions such that we can define
initialization behavior in the *reset* method and get the required values in the
*__call__* method.  This allows us to only lookup header information at the
start or only to do error checks and messages in the reset such that the
iteratively called *__call__* method does not need to spend time. Error messages
during the *__call__* phase typically also very quickly flood the log, a
separate *reset* method called at start can avoid thhis.


|           call back class   |     description           |
|-----------------------------|---------------------------|
| [ConfigCallback][betfsm.ConfigCallback] | Base class of callbacks for configuration.  Has a reset and __call__ method. The reset is called<br> every start of execution of the node (so, **not** in the constructor).|
| [CC_set_state][betfsm.CC_set_state] | Overrides the state parameter in the callback function with a state you specify in the <br>constructor.  Useful to delegate a callback to child nodes. |
| [CC_bb_table][betfsm.CC_bb_table]                 | Callback that returns a Numpy NDArray with columns specified by their header label  |
| [CC_bb_table_consume][betfsm.CC_bb_table_consume] | Callback that returns a numpy NDArray with columns specified by their header label <br> oldest rows $nrows$ are returned and removed|
| [CC_bb_array][betfsm.CC_bb_array]                 | Callback that returns a **copy** numpy NDArray with columns specified by index |
| [CC_bb_dict][betfsm.CC_bb_dict]                   | Callback that returns a **reference** to a dict at the given blackboard location|
| [CC_array][betfsm.CC_array]                       | Callback that returns a **reference** to the numpy array that was passed in his constructor|


Basic concepts:

  - The **table** in the name refers to a data-structure in the blackboard consisting of an 'header' with an ordered list of labels and a 'data' with an Numpy array. Data is stored in
    a circular buffer (with thread-safety lock) implement on top of an Numpy array.  They returns a **copy** of the data since
    other threads could remove or add data. 
  - The **array** in the name refers to a numpy array that is directly stored at the indicated path. They return a **reference** of 
  the data to maintain efficiency.  
  - The **dict**  in the name refers to a python dict that is directly stored at the indicated path. It returns a **reference** to the dict.
  - **bb** in the name means that the information that the callback returns comes from the blackboard at a given path
  - no **bb** in the name means that the information is directly passed to the constructor of the class and then returned by the callback


The blackboard-related callbacks take (at least) a *state* and *path* in their constructor.  These are used to resolve a location in the blackboard.
The path can also consist of relative paths, and these relative paths are defined relative to a BeTFSM node *state*.  In the practical implementation
of this the storage is in the blackboard["local"] [uid of the state].  Users should not access these locations directly but use the [get_path][betfsm.get_path] 
routine for this purpose. This function can also create non-existing entries in the blackboard. 


**Comments on the copy/reference semantics of the different callbacks are welcome.**



::: betfsm.ConfigCallback
    options:
      show_source: false
      show_root_heading: true

::: betfsm.CC_set_state
    options:
      show_source: false
      show_root_heading: true

::: betfsm.CC_bb_table
    options:
      show_source: false
      show_root_heading: true

::: betfsm.CC_bb_table_consume
    options:
      show_source: false
      show_root_heading: true

::: betfsm.CC_bb_array
    options:
      show_source: false
      show_root_heading: true

::: betfsm.CC_array
    options:
      show_source: false
      show_root_heading: true

::: betfsm.CC_bb_dict
    options:
      show_source: false
      show_root_heading: true
