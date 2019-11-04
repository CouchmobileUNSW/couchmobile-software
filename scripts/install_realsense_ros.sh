#!/bin/bash

if [[ -f install_lock || $1 == -f ]]; then
    # Check if in right directory. This should be run in root of workspace.
    if [ -d "src" ]; then
        cd src
        if [ -d "realsense-ros" ]; then
            # install realsense
            sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
            sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
            sudo apt-get install librealsense2-dkms
            sudo apt-get install librealsense2-utils
            sudo apt-get install librealsense2-dev
            sudo apt-get install librealsense2-dbg
            
            git clone https://github.com/IntelRealSense/realsense-ros.git
            cd realsense-ros/
            git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
            cd ..
        fi
        cd ..
        sudo apt-get install ros-melodic-ddynamic-reconfigure
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
