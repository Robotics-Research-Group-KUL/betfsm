#!/bin/bash

#
# Script to show the documentation locally
#
# creates a Python virtual environment if it does not exists.
#
# It could be necessary to do:
#    sudo apt-get install python3-venv 
# in order to be able to create python virtual environments.
#
# E. Aertbelien (2024)


echo -e "Script to show the documentation locally\n\n"
BASE=$(realpath --physical $(dirname "$0"))
if ! test -d "$BASE/venv"; then
    echo "creating virtual environment..."
    python3 -m venv "$BASE/venv"
    "$BASE/venv/bin/pip" install -r requirements.txt
else
    echo "reusing existing virtual environment"
fi

$BASE/venv/bin/mkdocs serve -a 0.0.0.0:8001 -w ../betfsm  -o
