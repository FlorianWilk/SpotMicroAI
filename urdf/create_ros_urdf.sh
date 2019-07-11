#!/bin/sh
# This script is currently needed for the ROS-Impl
sed 's/\.\.\/Parts\//package:\/\/spotmicroai\/urdf\/stl\//g' spotmicroai_gen.urdf.xml > spotmicroai_gen_ros.urdf
sed -i 's/\.\.\/STL\/files\//package:\/\/spotmicroai\/urdf\/stl\//g' spotmicroai_gen_ros.urdf
sed -i 's/\.\.\/STL\/lidar\//package:\/\/spotmicroai\/urdf\/stl\//g' spotmicroai_gen_ros.urdf

