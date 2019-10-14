#!/bin/bash

if [[ -f install_lock || $1 == -f ]]; then
    # Check if in right directory. This should be run in root of workspace.
    if [ -d "src" ]; then
        cd src
        # Perform installation instructions
        git clone https://github.com/ros-drivers/urg_node.git
        git clone https://github.com/ros-perception/laser_proc.git
        git clone https://github.com/ros-drivers/urg_c
        cd ..
        catkin_make --make-args -j4 -l2
    else
        echo "This file needs to be run in root of workspace."
    fi
else
    # Install lock doesn't exist; don't install here
    echo "This file must be called by the install script. Use -f to override."
fi
