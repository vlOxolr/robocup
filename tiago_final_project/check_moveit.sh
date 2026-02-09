#!/bin/bash

echo "========================================="
echo "Checking MoveIt Configuration"
echo "========================================="
echo ""

echo "1. Checking if roscore is running..."
if pgrep -x "rosmaster" > /dev/null; then
    echo "✓ roscore is running"
else
    echo "✗ roscore is NOT running"
    echo "  Please run: roscore"
    exit 1
fi
echo ""

echo "2. Checking if move_group is running..."
if rosnode list 2>/dev/null | grep -q "move_group"; then
    echo "✓ move_group is running"
    MOVE_GROUP_RUNNING=1
else
    echo "✗ move_group is NOT running"
    echo "  Please run: roslaunch tiago_moveit_config move_group.launch"
    MOVE_GROUP_RUNNING=0
fi
echo ""

if [ $MOVE_GROUP_RUNNING -eq 1 ]; then
    echo "3. Checking available planning groups..."
    echo "   Querying ROS parameters..."

    # 从参数服务器获取planning groups
    GROUPS=$(rosparam get /move_group/planning_pipelines/ompl/planning_plugin 2>/dev/null)

    if [ $? -eq 0 ]; then
        echo "   Available groups from robot_description_planning:"
        rosparam list | grep "robot_description_planning" | grep -E "group_name|^  [a-z_]+:" | head -20
    else
        echo "   Could not retrieve planning groups from parameter server"
    fi
    echo ""

    echo "4. Checking available action servers..."
    rostopic list | grep -E "move_group|pickup|place" | grep -v "cancel\|result\|status\|feedback"
    echo ""
fi

echo "5. Checking robot_description..."
if rosparam get /robot_description >/dev/null 2>&1; then
    echo "✓ robot_description is loaded"
else
    echo "✗ robot_description is NOT loaded"
fi
echo ""

echo "========================================="
echo "Diagnostics complete"
echo "========================================="
echo ""
echo "To run grasp planning, ensure:"
echo "1. roscore is running"
echo "2. Move_group is running: roslaunch tiago_moveit_config move_group.launch"
echo "3. Then run: rosrun manipulation grasp_planning.py"
