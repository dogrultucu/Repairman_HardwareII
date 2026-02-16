#!/bin/bash
# Quick verification script for Repairman Robot pipeline
# Usage: bash verify_setup.sh

set -e

WORKSPACE_DIR="${HOME}/repairman_ws"

echo "=========================================="
echo "Repairman Robot - Setup Verification"
echo "=========================================="
echo ""

# 1. Check workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ Workspace not found: $WORKSPACE_DIR"
    exit 1
fi
echo "✅ Workspace found: $WORKSPACE_DIR"

# 2. Check required directories
DIRS=(
    "src/repairman_vision"
    "src/repairman_vision/launch"
    "data"
    "weights"
    "build"
    "install"
)

for dir in "${DIRS[@]}"; do
    if [ -d "$WORKSPACE_DIR/$dir" ]; then
        echo "✅ Directory exists: $dir"
    else
        echo "⚠️  Missing directory: $dir (may be created after first build)"
    fi
done

# 3. Check key files
FILES=(
    "src/repairman_vision/package.xml"
    "src/repairman_vision/setup.py"
    "src/repairman_vision/launch/launch.py"
    "src/repairman_vision/repairman_vision/image_pub.py"
    "src/repairman_vision/repairman_vision/yolo_node.py"
    "src/repairman_vision/repairman_vision/toolpath_node.py"
    "src/repairman_vision/repairman_vision/execute_node.py"
    "src/repairman_vision/repairman_vision/verify_node.py"
    "src/repairman_vision/repairman_vision/state_manager.py"
    "data/test.png"
    "weights/best.pt"
)

echo ""
echo "Checking key files:"
for file in "${FILES[@]}"; do
    if [ -f "$WORKSPACE_DIR/$file" ]; then
        echo "✅ $file"
    else
        echo "❌ Missing: $file"
    fi
done

# 4. Check ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo ""
    echo "⚠️  ROS 2 not sourced. Run:"
    echo "   source /opt/ros/jazzy/setup.bash"
else
    echo ""
    echo "✅ ROS 2 Distro: $ROS_DISTRO"
fi

# 5. Check Python packages
echo ""
echo "Checking Python packages:"
PACKAGES=(
    "rclpy"
    "cv_bridge"
    "opencv"
    "ultralytics"
)

for pkg in "${PACKAGES[@]}"; do
    if python3 -c "import ${pkg}" 2>/dev/null; then
        echo "✅ $pkg installed"
    else
        echo "⚠️  $pkg not found (install: pip install $pkg)"
    fi
done

# 6. Verify build
if [ -d "$WORKSPACE_DIR/install" ]; then
    echo ""
    echo "✅ Build artifacts found"
    echo "   Source workspace with: source $WORKSPACE_DIR/install/setup.bash"
else
    echo ""
    echo "⚠️  Build artifacts not found. Run:"
    echo "   cd $WORKSPACE_DIR"
    echo "   source /opt/ros/jazzy/setup.bash"
    echo "   colcon build --symlink-install --packages-select repairman_vision"
fi

echo ""
echo "=========================================="
echo "Verification complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source workspace: source $WORKSPACE_DIR/install/setup.bash"
echo "2. Launch pipeline: ros2 launch repairman_vision launch.py"
echo "3. In another terminal, record topics: ros2 bag record -a"
echo "4. See README_REPAIRMAN.md for detailed instructions"
echo ""
