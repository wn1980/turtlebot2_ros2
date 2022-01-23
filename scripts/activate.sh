#!/usr/bin/env bash

set -e

ros2 action send_goal --feedback /mobile_base/auto_docking_action kobuki_ros_interfaces/action/AutoDocking {}
