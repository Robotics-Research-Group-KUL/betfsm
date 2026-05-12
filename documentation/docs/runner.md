### Running BeTFSM


Executing a state machine is done using a *Runner*.  This Runner iteratively executes
the state machine at a fixed frequency and returns when the state machine gives an
outcome not equal to TICKING.


Most of the parameters are defined by **RunnerBase**.  The actual runners inherit from this
base class and implement the environment related aspects such as timing, scheduling etc. There
are currently two runners **Runner** only depends on a pure Python environment, and
**ROSRunner** depends on a ROS2 environment and allow to use BeTFSM to implement a proper ROS2 node.

Besides their constructor arguments, all the runners will interpret command line arguments that allow all users to further configure
the execution.  Using the **--help** command line argument you can get a full list:

```python linenums="0"
--8<-- "betfsm/betfsm_examples/example_runner.txt"
```

You can also use the command-line arguments not to execute, but to **generate a .dot** file, in [GraphViz](https://graphviz.org/) format,
or to **generate a .json** file that can be used for further analysis of the BeTFSM-tree.  Currently the .json format does not contain
enough information to reconstruct the BeTFSM tree.

### RunnerBase

::: betfsm.RunnerBase
    options:
      show_source: false
      show_root_heading: true 


### Runner

::: betfsm.Runner
    options:
      show_source: false
      show_root_heading: true 

### ROSRunner

::: betfsm_ros.ROSRunner
    options:
      show_source: false
      show_root_heading: true 
