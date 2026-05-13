#!/bin/bash




task="up_and_down"
parameters=$(cat <<EOF
    {
        "delta_z" : 0.3
    }
EOF
)


ros2 action send_goal -f /task betfsm_interfaces/action/Task "
    task: $task
    parameters: '$parameters'
"

