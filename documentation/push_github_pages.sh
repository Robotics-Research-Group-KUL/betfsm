#!/bin/bash

#
# Script to show to push to github
#
# E. Aertbelien (2024)


echo -e "Script to show the documentation locally\n\n"
BASE=$(realpath --physical $(dirname "$0"))

echo "This script pushes the documentation to the github pages belonging to this repository"
echo "It is advised to check using the ./browse.sh script that everything is all right"
read -p "Are you sure? (y/n)" answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    	echo "Proceeding..."
	if ! test -d "$BASE/venv"; then
    		echo "creating virtual environment..."
    		python3 -m venv "$BASE/venv"
    		"$BASE/venv/bin/pip" install -r requirements.txt
	else
    		echo "reusing existing virtual environment"
	fi
    	"$BASE/venv/bin/mkdocs" gh-deploy
else
    echo "Cancelled."
fi





