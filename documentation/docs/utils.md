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

## Utility functions related to ROS2/environment package references

     

::: betfsm_ros.expand_package_ref
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 

::: betfsm_ros.expand_env_ref
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true       


::: betfsm_ros.expand_ref
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true             

## Logger related:

::: betfsm.logger.set_logger
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 

::: betfsm.logger.get_logger
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true  


## making a list of outcomes unique
::: betfsm.cleanup_outcomes
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true


## blackboard related
### Hierarchical Path Syntax for Blackboard Access

The blackboard is a nested structure composed of dictionaries and lists.
Values inside this structure can be accessed or modified using a path
string, where components are separated by "/" (e.g. "users/list/0").

This extended syntax supports:
- dictionary navigation
- list indexing
- list operations (length, append, insert, delete, pop)
All operators are chosen to be URL‑safe and easy to type directly in a browser.

### Basic rules

- Each path component is separated by "/".
- Dictionary keys are addressed by name:
      "config/system/mode"
- List elements are addressed by numeric indices:
      "users/0/name"


### List Operators 

The following operators apply when the current node is a list:

-  "~len"
      Returns the length of the list.
      Example: "users/~len"

-  "~append"
      Appends a new value to the list.
      Example: "users/~append"

-  "~insert:N"
      Inserts a value at index N.
      Example: "users/~insert:2"

-  "~del:N"
      Deletes the element at index N.
      Example: "users/~del:1"

-  "~pop"
      Removes the last element
      Example: "users/~pop"

-  "N"   (digit)
      Replaces or retrieves the element at index N.
      Example: "users/3"

### Examples
```python
    #  Get a nested dictionary value:
    get_path_value(bb, "settings/ui/theme")

    # Get the third item in a list:
    get_path_value(bb, "users/2")

    # Get the length of a list:
    get_path_value(bb, "users/~len")

    # Append a new user:
    set_path_value(bb, "users/~append", {"name": "Alice"})

    #Insert at index 1:
    set_path_value(bb, "users/~insert:1", {"name": "Bob"})

    # Replace element at index 3:
    set_path_value(bb, "users/3", {"name": "Charlie"})

    # Delete element at index 0:
    set_path_value(bb, "users/~del:0", None)
```
### Notes

- All operators are ASCII‑only and require no URL encoding.
- Intermediate dictionaries or lists are created automatically when needed.
- When replacing a list element by index, the list is auto‑extended with
  None values if the index is beyond the current length.




::: betfsm.dumps_blackboard
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true  

      
::: betfsm.get_path_value
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 

::: betfsm.set_path_value
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 


::: betfsm.get_path_location
    options:
      heading_level: 3
      show_source: false
      show_root_heading: true 