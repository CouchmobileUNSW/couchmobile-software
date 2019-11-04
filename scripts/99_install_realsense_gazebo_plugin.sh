#!/bin/bash

if [[ -f install_lock || $1 == -f ]]; then
    # Check if in right directory. This should be run in root of workspace.
    if [ -d "src" ]; then
        cd src
        # Perform installation instructions
        git clone https://github.com/SyrianSpock/realsense_gazebo_plugin.git

        # Install required dependencies
        sudo apt install libgtest-dev
        cd /usr/src/googletest
        sudo cmake .
        sudo cmake --build . --target install
        
        # Make edit to CMakefile to correctly find Gtesti
        cd realsense_gazebo_plugin
        sed '/^find_package(Threads REQUIRED).*/a find_package(GTest REQUIRED)' CMakeLists.txt

    else
        echo "This file needs to be run in root of workspace."
    fi
else
    # Install lock doesn't exist; don't install here
    echo "This file must be called by the install script. Use -f to override."
fi
