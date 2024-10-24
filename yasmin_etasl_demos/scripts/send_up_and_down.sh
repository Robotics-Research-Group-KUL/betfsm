#!/bin/bash




task="up_and_down"
parameters=$(cat <<EOF
    {
    }
EOF
)


ros2 action send_goal -f /task yasmin_etasl_interfaces/action/Task "
    task: $task
    parameters: '$parameters'
"

