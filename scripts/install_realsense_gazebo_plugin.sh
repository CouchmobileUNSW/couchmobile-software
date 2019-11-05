#!/bin/bash

if [[ -f install_lock || $1 == -f ]]; then
    # Check if in right directory. This should be run in root of workspace.
    if [ -d "src" ]; then
        pushd src
        # Perform installation instructions

        git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
        # Remove useless crap
        cd realsense_gazebo_plugin/src/
        sed -i 's/this->world->GetName()/this->world->Name()/g' RealSensePlugin.cpp 
        sed -i 's/this->world->GetSimTime()/this->world->SimTime()/g' RealSensePlugin.cpp
        sed -i 's/this->world->GetSimTime()/this->world->SimTime()/g' gazebo_ros_realsense.cpp


        # Install required dependencies
        sudo apt install libgtest-dev
        pushd /usr/src/googletest
        sudo cmake .
        sudo cmake --build . --target install
        popd

        # Make edit to CMakefile to correctly find Gtesti
        #cd realsense_gazebo_plugin
        #sed '/^find_package(Threads REQUIRED).*/a find_package(GTest REQUIRED)' CMakeLists.txt
        
        popd
        catkin_make
    else
        echo "This file needs to be run in root of workspace."
    fi
else
    # Install lock doesn't exist; don't install here
    echo "This file must be called by the install script. Use -f to override."
fi
