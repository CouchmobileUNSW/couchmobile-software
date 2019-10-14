#!/bin/bash

if [[ -f install_lock || $1 == -f ]]; then
    # Check if in right directory. This should be run in root of workspace.
    if [ -d "src" ]; then
        cd src
        if [ -d "realsense-ros" ]; then
            git clone https://github.com/IntelRealSense/realsense-ros.git
            cd realsense-ros/
            git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
            cd ..
        fi
        cd ..
        catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
        --make-args -j4 -l2
        catkin_make install
    else
        echo "This file needs to be run in root of workspace."
    fi
else
    # Install lock doesn't exist; don't install here
    echo "This file must be called by the install script. Use -f to override."
fi
