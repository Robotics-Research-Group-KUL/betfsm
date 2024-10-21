# Utilities

## Logging


Logging is implemented in `logging.py`.

Some logging mechanism is foreseen between this library and the logging routines
of ROS2.  This allows us to do two things:

  - logging to console when ROS2 is not present.
  - offer some granularity with logging classes by using categories. Code that logs gets a log object by
  specifying a category.  Uses can set the logger that belongs to a category. Example of categories are
  `default`, `state` (logging entering and leaving of each state), `service` (logging of service calls to ROS2)

See the documentation of `set_logger()` below.



For example, your main could contain:
```python
set_logger("default",my_node.get_logger())
set_logger("service",my_node.get_logger())
set_logger("state",my_node.get_logger())
```
This sets the ROS2 logger for all of the above categories.

## Utility functions

     

::: yasmin_etasl.yasmin_ticking_ros.expand_package_ref
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 

::: yasmin_etasl.yasmin_ticking_ros.expand_env_ref
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true       


::: yasmin_etasl.yasmin_ticking_ros.expand_ref
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true             

::: yasmin_etasl.logger.set_logger
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 

::: yasmin_etasl.logger.get_logger
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true  

::: yasmin_etasl.yasmin_ticking.cleanup_outcomes
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true

::: yasmin_etasl.yasmin_ticking_ros.dumps_blackboard
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true  

      