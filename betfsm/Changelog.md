# Changelog

## [1.1.0] [in development]

### Changed

- restructuring of directory structure such that we will have different
packages instead of one monolitic package.  This will be  a python-only
pakcage betfsm, a ROS2 package betfsm_ros, and a eTaSL/crospi interface
in betfsm_crospi / betfsm_interfaces.

- made graphviz visitor more robust against names of the nodes with spaces in
or strange characters.

- more accurate timing of BeTFSMRunner, BeTFSMRunner now returns an outcome
(as was originally intended but not done), added debugging options and options
to display the active nodes.

- significantly improved graphical user interface:  
    - It is used by replacing the runner with BeTFSMRunnerGUI.  
    - The graphviz-based monitoring GUI is removed and replaced by a JSON-format.  
    - The active nodes are streamed separately using websockets.    
    - The GUI allows the user to expand/collapse the BeTFSM tree. There
      is also an auto-expand feature that only expands branches that contain active nodes.
      The GUI can deal with large trees.  
    - The front-end of the GUI keeps trying to connect to the server every 5 seconds. This
      allows the BeTFSM server to be shutdown and restarted and the GUI will automatically
      picks this up.

- each BeTFSM node has now a UUID, a parent.  There is now logging of the
active nodes in two ways:  using ```_global_log``` class variable to have
an instantaneous view on the active nodes, and using ```_global_publish_log```
class variable to have a view over the active nodes within a certain period. 
This last is handy for the GUI, since it tracks also nodes that are activated/deactivated
within one tick.

- improved error checking to guard against user programming errors and to decrease
debugging time.

- added various examples and tutorial documentation using these examples.




## [1.0.0] - 2025-11-1
### Added
- initial version, used in robotics applications in combination with
etaslros2/crospi
- initial GUI
- ActionServer
- ROS2 bridge
- eTaSL / Crospi bridge






## Types of changes: Added Changed Fixed Removed Deprecated Security
