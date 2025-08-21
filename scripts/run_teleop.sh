#!/bin/bash
# Simple teleop launcher script

cd /home/jetracer/ros2_ws
source install/setup.sh

echo "Starting teleop_twist_keyboard..."
echo "Use the following keys to control the robot:"
echo "  i - Forward"
echo "  , - Backward" 
echo "  j - Turn left"
echo "  l - Turn right"
echo "  k - Stop"
echo "  q/z - Increase/decrease linear speed"
echo "  w/x - Increase/decrease angular speed"
echo ""
echo "Press CTRL+C to exit"
echo ""

ros2 run teleop_twist_keyboard teleop_twist_keyboard
