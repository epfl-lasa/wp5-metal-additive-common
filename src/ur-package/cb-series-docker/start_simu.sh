#!/bin/bash

# Terminal 1: Run the first command in /docker1
gnome-terminal -e "bash -c \"ls; exec bash\""

gnome-terminal -- bash && cd docker_polyscope && ./start_docker.sh

gnome-terminal -- bash && cd docker_interface_ros && ./start_docker.sh; exec bash


