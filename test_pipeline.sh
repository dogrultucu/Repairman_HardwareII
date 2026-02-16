#!/bin/bash

# Repairman Robot - Test Pipeline Script
# Tüm components'i başlatır: Pipeline + RViz + Bag Record

set -e

WORKSPACE_DIR="${HOME}/repairman_ws"
cd "$WORKSPACE_DIR"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog}"
mkdir -p "$ROS_LOG_DIR"

# ROS 2 environment'ı source et
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=================================================="
echo "🤖 Repairman Robot - Full Test Pipeline"
echo "=================================================="
echo ""
echo "✅ Environment sourced"
echo "📁 Workspace: $WORKSPACE_DIR"
echo ""

RVIZ_CONFIG="$WORKSPACE_DIR/src/repairman_vision/rviz/repairman_pipeline.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "❌ RViz config not found: $RVIZ_CONFIG"
    exit 1
fi

# Check if nodes are available
echo "Checking if nodes are built..."
if ! ros2 pkg executables repairman_vision > /dev/null 2>&1; then
    echo "❌ Nodes not found! Run:"
    echo "   colcon build --symlink-install --packages-select repairman_vision"
    exit 1
fi
echo "✅ All nodes available"
echo ""

# Kill any existing processes
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "ros2 bag record" 2>/dev/null || true
sleep 2

echo "🚀 STEP 1: Launching pipeline nodes (image_pub, yolo_node, toolpath_node, execute_node, verify_node, state_manager)"
echo "   This will take ~10 seconds to initialize YOLO..."
echo ""

# Terminal 1: Launch pipeline
ros2 launch repairman_vision launch.py > /tmp/pipeline.log 2>&1 &
LAUNCH_PID=$!
echo "   Pipeline PID: $LAUNCH_PID"

# Wait for nodes to be ready
sleep 12

# Check if nodes are running
NODES=$(ros2 node list 2>/dev/null | wc -l)
if [ "$NODES" -lt 6 ]; then
    echo ""
    echo "⚠️  Only $NODES nodes found (expected 6+). Checking logs..."
    tail -30 /tmp/pipeline.log
fi
echo "   ✅ Pipeline nodes started"
echo ""

echo "📊 STEP 2: Launching RViz visualization..."
rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "   RViz PID: $RVIZ_PID"
sleep 3
echo "   ✅ RViz launched (you should see it on screen)"
echo ""

echo "📝 STEP 3: Recording topics for 45 seconds..."
echo "   Topics: /camera/image_raw, /damage/mask, /repair/toolpath, /repair/executed, /repair/quality_score"
echo ""

# Terminal 3: Record bag
mkdir -p "$WORKSPACE_DIR/test_runs"
BAG_NAME="test_run_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="$WORKSPACE_DIR/test_runs/$BAG_NAME"

ros2 bag record \
    /camera/image_raw \
    /damage/mask \
    /damage/annotated \
    /repair/toolpath \
    /repair/executed \
    /repair/executed_path \
    /repair/quality_score \
    /repair/state \
    -o "$BAG_PATH" > /tmp/bag.log 2>&1 &
BAG_PID=$!
echo "   Bag PID: $BAG_PID"
echo "   Recording to: $BAG_PATH"

# Progress indicator
echo ""
for i in {1..45}; do
    REMAINING=$((45 - i))
    echo -ne "   Recording... $REMAINING seconds remaining\r"
    sleep 1
done
echo "   Recording complete!                           "

# Stop bag recording
kill $BAG_PID 2>/dev/null || true
sleep 2

echo ""
echo "=================================================="
echo "✅ TEST COMPLETE"
echo "=================================================="
echo ""

# Analyze bag
echo "📊 Analyzing recorded topics..."
ros2 bag info "$BAG_PATH" 2>/dev/null | grep -E "(Topic:|Count:)" | head -20

echo ""
echo "=================================================="
echo "📋 RESULTS"
echo "=================================================="
echo ""
echo "Bag location: $BAG_PATH"
echo ""
echo "Next steps:"
echo ""
echo "1. Check RViz window (still open):"
echo "   - See camera image (top-left)"
echo "   - See damage mask (top-middle)"
echo "   - See YOLO annotated (top-right)"
echo "   - See green execution path (3D view, center)"
echo ""

echo "2. Analyze the recorded bag:"
echo "   ros2 bag info $BAG_PATH"
echo ""

echo "3. Replay the bag (in new terminal):"
echo "   ros2 bag play $BAG_PATH"
echo ""

echo "4. To stop everything:"
echo "   Press Ctrl+C in this terminal"
echo ""

echo "=================================================="
echo "Pipeline still running. Press Ctrl+C to stop all."
echo "=================================================="
echo ""

# Keep pipeline running
wait $LAUNCH_PID $RVIZ_PID

echo ""
echo "🛑 Pipeline stopped."
