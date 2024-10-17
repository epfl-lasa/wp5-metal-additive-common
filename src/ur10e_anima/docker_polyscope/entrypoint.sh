#!/bin/bash

# Copy urcaps into bundle, to be installed properly, when the simulator is started
cp -r /urcaps/*.jar /ursim/GUI/bundle/ 2>/dev/null

# Execute URSim
/ursim/start-ursim.sh ${ROBOT_MODEL}
