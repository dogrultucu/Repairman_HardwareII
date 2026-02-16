#!/bin/bash

# Repairman Robot - Simple Test Script
# Check if everything works

set -e

WORKSPACE_DIR="${HOME}/repairman_ws"
cd "$WORKSPACE_DIR"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"
mkdir -p "$ROS_LOG_DIR"

LAUNCH_PID=""
cleanup() {
    if [ -n "$LAUNCH_PID" ]; then
        kill "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Source ROS
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo ""
echo "=================================================="
echo "🤖 Repairman Robot - Quick Diagnostics"
echo "=================================================="
echo ""

RVIZ_CONFIG="$WORKSPACE_DIR/src/repairman_vision/rviz/repairman_pipeline.rviz"

# 0. Run quick logic/style checks
echo "Step 0: Running quick tests..."
pytest -q \
  src/repairman_vision/test/test_state_manager_logic.py \
  src/repairman_vision/test/test_pep257.py
echo "✅ Quick tests passed"
echo ""

# 1. Check if launch file exists
echo "Step 1: Checking launch file..."
if [ -f "$WORKSPACE_DIR/install/repairman_vision/share/repairman_vision/launch/launch.py" ]; then
    echo "✅ Launch file installed correctly"
else
    echo "❌ Launch file NOT found. Checking share directory..."
    ls -la "$WORKSPACE_DIR/install/repairman_vision/share/repairman_vision/" 2>/dev/null || echo "Share dir missing!"
fi
echo ""

echo "Step 1.1: Checking RViz config..."
if [ -f "$RVIZ_CONFIG" ]; then
    echo "✅ RViz config found: $RVIZ_CONFIG"
else
    echo "❌ RViz config missing: $RVIZ_CONFIG"
    exit 1
fi
echo ""

# 2. Check executable nodes
echo "Step 2: Checking available executables..."
ros2 pkg executables repairman_vision
echo ""

# 3. Check topics
echo "Step 3: Starting pipeline in background (15 sec warmup)..."
ros2 launch repairman_vision launch.py > /tmp/test.log 2>&1 &
LAUNCH_PID=$!
echo "   Launched with PID: $LAUNCH_PID"
sleep 15

echo ""
echo "Step 4: Checking active nodes..."
echo "   Active nodes:"
ros2 node list 2>/dev/null | sort || echo "   (nodes still initializing)"
echo ""

echo "Step 5: Checking published topics..."
echo "   Published topics:"
ros2 topic list 2>/dev/null | sort || echo "   (topics still initializing)"
echo ""

echo "Step 5.1: Checking robot model stream..."
if ros2 topic echo /robot_description --once >/tmp/robot_description_check.log 2>&1; then
    echo "   ✅ /robot_description is available (RobotModel can render)"
else
    echo "   ⚠️  /robot_description not available (RViz RobotModel cannot render the robot)"
fi
echo ""

echo "Step 6: Checking topic message counts (this may take a few seconds)..."
for topic in /camera/image_raw /damage/mask /repair/toolpath /repair/executed_path /repair/executed /repair/quality_score /repair/state; do
    COUNT=$(ros2 topic echo "$topic" --once 2>/dev/null | grep -c "." || echo "0")
    if [ "$COUNT" -gt 0 ]; then
        echo "   ✅ $topic (active)"
    else
        echo "   ⚠️  $topic (no message)"
    fi
done
echo ""

# Clean up
echo "Stopping pipeline..."
kill $LAUNCH_PID 2>/dev/null || true
LAUNCH_PID=""
sleep 2

echo ""
echo "=================================================="
echo "🎯 SUMMARY"
echo "=================================================="
echo ""
echo "If you see:"
echo "  ✅ /camera/image_raw"
echo "  ✅ /damage/mask" 
echo "  ✅ /repair/toolpath"
echo "  ✅ /repair/executed_path (GREEN LINE IN RVIZ!)"
echo ""
echo "Then everything is working! 🎉"
echo ""
echo "Next: Run in 3 separate terminals:"
echo ""
echo "  Terminal 1:"
echo "    cd ~/repairman_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "    ros2 launch repairman_vision launch.py"
echo ""
echo "  Terminal 2:"
echo "    cd ~/repairman_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "    rviz2 -d $RVIZ_CONFIG"
echo ""
echo "  Terminal 3:"
echo "    cd ~/repairman_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "    ros2 bag record -a -o my_test"
echo ""
echo "=================================================="
echo ""
