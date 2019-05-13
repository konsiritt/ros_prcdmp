#!/bin/sh -e

IP="$1"
ROOT=$(dirname "$(realpath $0)")
cd ${ROOT}

### make ###
cd ${ROOT}/../../..
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFranka_DIR:PATH=../../libfranka/build

### push all build files of the project ###

cd ${ROOT}/../../../..

if [ ! -z "$1" ];
then
rsync -r franka_ros_catkin/src franka_adm@$IP:~/franka_ros_catkin/
rsync -r franka_ros_catkin/build franka_adm@$IP:~/franka_ros_catkin/
rsync -r franka_ros_catkin/devel/lib franka_adm@$IP:~/franka_ros_catkin/devel/
rsync -r franka_ros_catkin/devel/share franka_adm@$IP:~/franka_ros_catkin/devel/ 
rsync -r franka_ros_catkin/devel/include franka_adm@$IP:~/franka_ros_catkin/devel/ 
fi
