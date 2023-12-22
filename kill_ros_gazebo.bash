#!/usr/bin/env bash

RED='\033[0;31m'
GREEN='\033[1;32m'
# ON_GREEN='\033[42m'

echo -e ${GREEN}'Before kill:'
ps -ef | grep gazebo
ps -ef | grep ros
ps -ef | grep 'python3 src/scripts/run.py'


echo -e ${GREEN}'Killing gazebo and ros ...'
pkill -9 -f 'gazebo'
pkill -9 -f 'ros'
pkill -9 -f 'python3 src/scripts/run.py'


echo -e ${GREEN}"After kill:"
ps -ef | grep gazebo
ps -ef | grep ros
ps -ef | grep 'python3 src/scripts/run.py'

