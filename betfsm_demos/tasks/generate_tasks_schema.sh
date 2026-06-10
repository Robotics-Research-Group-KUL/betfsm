#!/usr/bin/env bash

echo "Generating json files for the .etasl.lua files and create a tasks-schema.json file to assembling references to this tasks"
ros2 run betfsm_crospi generate_schemas create tasks-schema.json ./betfsm_demos_lib crospi_application_template robot_models/robot_specifications --flat

echo " "
echo "Add references form another ROS2 package (crospi_application_template) to tasks-schema.json"
ros2 run betfsm_crospi generate_schemas add tasks-schema.json crospi_application_template task_specifications/libraries

echo " "
echo "add this generate file to your .gitignore, it depends on your own directory structure and should not be commited to git"
echo " "
echo "If you now want to make a new list of tasks to load in your BeTFSM, start with"
echo "{ \"\$schema\": \"tasks-schema.json\" } and use autocompletion" 