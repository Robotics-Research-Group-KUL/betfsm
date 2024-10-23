#!/bin/bash 
ros2 topic pub --rate 3 /my_topic etasl_interfaces/msg/Output "{names: ['a','b'], data: [2,3]}" 
