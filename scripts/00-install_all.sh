#!/bin/bash

SCRIPTS_DIR=$PWD

cd ../../..

if [ -d "src" ]; then
    # Create install lock
    touch install_lock
    
    # Call all install scripts
    for filename in $SCRIPTS_DIR/install*; do
        ( $filename )
    done

    # Remove install lock
    rm install_lock

    if grep -Fxq "source $PWD/devel/setup.bash" ~/.bashrc then
        echo "source $PWD/devel/setup.bash" >> ~/.bashrc
    fi

    source ~/.bashrc
else
    echo "Root directory of workspace could not be found. This script needs to
    be run from WorkspaceRoot/src/couchmobile-software/scripts/"
fi
