#!/bin/bash

if [[ -f install_lock || $1 == -f ]]; then
    # Check if in right directory. This should be run in root of workspace.
    if [ -d "src" ]; then
        cd src
        # Perform installation instructions
        git clone https://github.com/ros-drivers/rosserial.git
        sudo apt-get install ros-melodic-rosserial-arduino
        sudo apt-get install ros-melodic-rosserial
        cd ..
        catkin_make
        catkin_make install
        mkdir -p ~/Arduino/libraries
        cd ~/Arduino/libraries
        rm -rf ros_lib
        rosrun rosserial_arduino make_libraries.py .
    else
        echo "This file needs to be run in root of workspace."
    fi
else
    # Install lock doesn't exist; don't install here
    echo "This file must be called by the install script. Use -f to override."
fi
