# Logging in BeTFSM



You can easily log by calling [get_logger().info("my_message")][betfsm.get_logger]
You can also specify a category using [get_logger("my_lib").info("my message")][betfsm.get_logger].
Categories are used to turn on/turn off the logging for a given categories.


The logging functions have also a severity, i.e. in order: `debug`, `info`, `warn`, `error`,`fatal`.

Each category can be associated with a Logger class that directly prints to console or connects to a logging system of a larger framework. The purpose of the logging classes in BeTFSM is just to connect to existing systems, not to provide an elaborate framework itself.  Some tools or environments can use this severity for additional filtering and display (such as ROS2).

The logger for a given category can be specified using [set_logger(category, logger_class)][betfsm.set_logger].  You can use ROS2's [node.get_logger()](https://docs.ros.org/en/rolling/Tutorials/Demos/Logging-and-logger-configuration.html) as a directly as logger class, or the [DummyLogPrinter][betfsm.DummyLogPrinter] or [LogPrinter][betfsm.LogPrinter] that BeTFSM provides.  


!!! Note
    If you use one of the [Runner][betfsm.Runner] or [RosRunner][betfsm_ros.ROSRunner] classes, you can activate categories from **the
    command-line**.

!!! Warning    
    There is a 'constructor' logger that logs the constructor calls when you are defining your BeTFSM, this can
    be useful to trace back if you have exceptions raised during construction, deep in your hierarchy.
    Since it involves construction of the BeTFSM tree, it has to be set before this constructions. Therefore it cannot be set
    using the Runner command-line parameters.

## Categories

The BeTFSM libraries and (possibly) user can (optionally) declare the logging
categories they use, by calling in their library the `add_logger_category(my_cat)` 
functions.  Users (or e.g. the Runner command-line help) can get the available
categories using the `get_logger_categories()` function.



Known categories used in betfsm_crospi:

- 'default': the default logger, i.e. empty string: ([LogPrinter][betfsm.LogPrinter] to console)
- 'state'  : entry/exit of all BeTFSM nodes (run-time).
- 'constructor' : during construction of the BeTFSM tree.
- 'crospi' : crospi BeTFSM nodes.
- 'action' : related to the BeTFSM action client
- 'service': ROS2 service calls
- 'unknown': an unknown category for which there is no handler class defined. 

By default, the default logger is [LogPrinter][betfsm.LogPrinter] and and unknown is [DummyLogPrinter][betfsm.DummyLogPrinter], and no other categories have a Logger class specified.

## API


::: betfsm.get_logger

::: betfsm.set_logger


::: betfsm.DummyLogPrinter


::: betfsm.LogPrinter
