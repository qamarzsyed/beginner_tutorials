# Basic Publisher/Subscriber Node in ROS 2

## Overview
Basic publisher and subscriber that output and read a message to a topic in ROS 2
Files can be found in the cpp_pubsub directory in the src directory
Now also has a service in the publisher that can be used to change the message to default message

## Build and Run (start in root directory)
colcon build
. install/setup.bash

### Launch talker and listener with the terminal
run ros2 run cpp_pubsub talker in one terminal 
and run ros2 run cpp_pubsub listener in another terminal

### Using launch file
source ros2 package
ros2 launch cpp_pubsub launch.py

## Dependencies
ROS 2 Humble Hawksbill
