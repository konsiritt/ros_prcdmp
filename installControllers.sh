#!/bin/sh -e

IP="$1"
ROOT=$(dirname "$(realpath $0)")
cd ${ROOT}

### make ###
cd ${ROOT}/../../..
catkin_make

### push all build files of the project ###

cd ${ROOT}/../../../..

if [ ! -z "$1" ];
then
rsync -r franka_ros_catkin franka_adm@$IP:~/
fi
