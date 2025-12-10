#!/bin/bash

DIR="$(cd "$(dirname "$0")" && pwd)"

echo "This script pushes the documentation to the github pages belonging to this repository"
echo "It is advised to check using the ./browse.sh script that everything is all right"
read -p "Are you sure? (y/n)" answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    echo "Proceeding..."
    "$DIR/venv/bin/activate/mkdocs" gh-deploy
else
    echo "Cancelled."
fi
