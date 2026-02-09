#!/bin/bash

echo "========================================="
echo "Checking ROS Navigation System Status"
echo "========================================="
echo ""

echo "1. Checking if roscore is running..."
if pgrep -x "rosmaster" > /dev/null; then
    echo "✓ roscore is running"
else
    echo "✗ roscore is NOT running"
fi
echo ""

echo "2. Checking key navigation nodes..."
echo "   - map_server:"
rosnode info /map_server 2>/dev/null && echo "   ✓ Running" || echo "   ✗ NOT running"

echo "   - amcl:"
rosnode info /amcl 2>/dev/null && echo "   ✓ Running" || echo "   ✗ NOT running"

echo "   - move_base:"
rosnode info /move_base 2>/dev/null && echo "   ✓ Running" || echo "   ✗ NOT running"
echo ""

echo "3. Checking critical topics..."
echo "   - /map:"
rostopic info /map 2>/dev/null | grep -q "Type: nav_msgs/OccupancyGrid" && echo "   ✓ Publishing" || echo "   ✗ NOT publishing"

echo "   - /amcl_pose:"
rostopic info /amcl_pose 2>/dev/null | grep -q "Type: geometry_msgs/PoseWithCovarianceStamped" && echo "   ✓ Publishing" || echo "   ✗ NOT publishing"

echo "   - /move_base/goal:"
rostopic info /move_base/goal 2>/dev/null && echo "   ✓ Available" || echo "   ✗ NOT available"
echo ""

echo "4. Checking TF transforms..."
echo "   Checking map -> odom transform:"
rosrun tf tf_echo map odom 2>&1 | head -3
echo ""

echo "5. Checking move_base status..."
rostopic echo /move_base/status -n 1 2>/dev/null || echo "   ✗ No status available"
echo ""

echo "========================================="
echo "Diagnostics complete"
echo "========================================="
