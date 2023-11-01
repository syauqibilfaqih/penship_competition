#!/bin/bash

cd ~/vrx_ws
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=acoustic_tracking_task
