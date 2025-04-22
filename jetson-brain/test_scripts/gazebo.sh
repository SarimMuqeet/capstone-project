#!/bin/bash

# check if arch is arm64
if [ "$(uname -m)" = "aarch64" ]; then
  # check if src/nav2_bringup does not exist
  if [ ! -d "src/nav2_bringup" ]; then
    curl -L https://api.github.com/repos/ros-planning/navigation2/tarball/1.2.2 \
      | tar xz -C src/ --wildcards "*/nav2_bringup" --strip-components=1

    # remove "turtlebot3_gazebo" dependency from nav2_bringup/package.xml since it has not arm64 build
    sed -i '/turtlebot3_gazebo/d' src/nav2_bringup/package.xml
  fi
fi


#https://robotics.stackexchange.com/questions/103623/cannot-find-turtlebot3-gazebo-on-ros2-humble-on-22-04
#https://github.com/ros-navigation/navigation2/issues/3766#issuecomment-1697583063
