#!/bin/bash

cd ~/vrx_ws
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=scan_dock_deliver_task

