# Repairman Robot MVP - Build & Deploy Commands

## Complete Command Reference (Copy & Paste)

### Setup Phase (Do Once)

```bash
# 1. Navigate to workspace
cd ~/repairman_ws

# 2. Install system dependencies (if needed)
sudo apt-get update
sudo apt-get install -y python3-pip

# 3. Install Python packages
pip install opencv-python ultralytics

# 4. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 5. Build workspace
colcon build --symlink-install --packages-select repairman_vision

# 6. Source built workspace
source install/setup.bash

# 7. Verify setup
bash verify_setup.sh
```

**Expected output**: All checks pass ✅

---

## Run Phase (Daily Use)

### Terminal 1: Launch All Nodes (Single Command)

```bash
# First-time setup in this terminal
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch everything
ros2 launch repairman_vision launch.py
```

**You should see 6 nodes starting in ~5 seconds:**
```
[image_pub-1] ... Publishing test.png -> /camera/image_raw @ 5.0 Hz
[yolo_node-2] ... Loaded YOLO weights: ...
[toolpath_node-3] ... ToolpathNode subscribed: /damage/mask ...
[execute_node-4] ... ExecuteNode initialized
[verify_node-5] ... VerifyNode initialized
[state_manager-6] ... StateManager initialized
```

Let this run in the background.

---

### Terminal 2: Record Topics (Validation)

```bash
# In a new terminal
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Record all topics (run for 30-60 seconds)
ros2 bag record -a -o full_pipeline_bag

# Press Ctrl+C after 30-60 seconds
```

---

### Terminal 2: Verify (After Recording Stops)

```bash
# Inspect the recording
ros2 bag info full_pipeline_bag

# You should see something like:
# Files:             full_pipeline_bag_0.mcap
# Bag size:          X MiB
# Duration:          30.0s
# Messages:          ~4000+
#
# Topic: /camera/image_raw | Count: ~150
# Topic: /damage/mask | Count: ~150
# Topic: /damage/annotated | Count: ~150
# Topic: /repair/toolpath | Count: ~100+
# Topic: /repair/executed | Count: 1-3
# Topic: /repair/quality_score | Count: ~60+
```

✅ **If you see messages on all topics → Pipeline works!**

---

## Custom Configurations

### Use Custom Test Image

```bash
ros2 launch repairman_vision launch.py image_path:=/path/to/my/image.png
```

### Set Quality Threshold (0.0 = very strict, 1.0 = very lenient)

```bash
# More strict (require 70% improvement)
ros2 launch repairman_vision launch.py quality_threshold:=0.7

# More lenient (accept any improvement)
ros2 launch repairman_vision launch.py quality_threshold:=0.3
```

### Disable Auto-Start (Manual Cycle Control)

```bash
ros2 launch repairman_vision launch.py auto_start:=false
```

### Combine Multiple Parameters

```bash
ros2 launch repairman_vision launch.py \
  image_path:=~/my_image.png \
  quality_threshold:=0.6 \
  auto_start:=true
```

---

## Troubleshooting Commands

### Check if Nodes are Running

```bash
ros2 node list
```

Expected output:
```
/execute_node
/image_pub
/state_manager
/toolpath_node
/verify_node
/yolo_node
```

### Check Topic Publishing

```bash
# List all topics
ros2 topic list

# Show message counts on a topic (use bag record instead if daemon unstable)
ros2 topic hz /repair/toolpath

# Echo a message (may timeout with daemon issues)
ros2 topic echo /repair/toolpath --once
```

### Enable Debug Logging

```bash
export ROS_LOG_LEVEL=DEBUG
ros2 launch repairman_vision launch.py
```

### Clean Rebuild If Issues Persist

```bash
cd ~/repairman_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select repairman_vision
source install/setup.bash
ros2 launch repairman_vision launch.py
```

---

## Expected Behavior Timeline

### 0-10 seconds
- Nodes initializing
- Image_pub starts publishing frames
- YOLO loading weights (may take 5-10 sec)

### 10-30 seconds
- YOLO detects damage in test image
- Mask and annotated images publishing
- Toolpath being generated
- State machine enters REPAIR phase
- Execution simulator running

### 30-45 seconds
- Execution completes (`/repair/executed` published)
- State machine enters RESCAN
- Waits for re-scan image (simulated)
- Quality score computed

### 45-60 seconds
- State machine evaluates quality
- Decision: PASS or REPAIR_AGAIN (based on threshold)
- If PASS: returns to IDLE, waits for next cycle
- If REPAIR_AGAIN: loops back to REPAIR

### Repeat every 30 seconds
- (if `cycle_period_sec=30` in state_manager)

---

## Real-Time Monitoring (Advanced)

### Play Back Recording in Loop

```bash
# Terminal 3: Playback
ros2 bag play full_pipeline_bag --loop

# Terminal 4: Monitor specific topic during playback
ros2 topic echo /repair/quality_score
```

### Use RViz for Visualization (Optional)

```bash
# If you have a config file
rviz2 -d ~/repairman_ws/rviz_config.rviz

# Otherwise, launch RViz and manually add:
# - /fire/executed_path (nav_msgs/Path) → Green path line
# - Camera image topics → Raw image display
```

---

## Performance Expectations

| Metric | Expected | Notes |
|--------|----------|-------|
| Time IDLE→SCAN | <1 sec | Auto-trigger |
| Time SCAN→DETECT | 5-15 sec | YOLO inference on test image |
| Time DETECT→PLAN | <1 sec | Contour + downsampling |
| Time PLAN→REPAIR | 5-10 sec | Execution simulation (depends on point count) |
| Time REPAIR→RESCAN | 2 sec | Rescan delay (configurable) |
| Time RESCAN→EVALUATE | <1 sec | Quality computation |
| Time EVALUATE→PASS/RETRY | <1 sec | Decision logic |
| **Total cycle time** | ~15-30 sec | Single pass, no retries |

**With retry** (quality < threshold, max_repair_passes=2):
- Add another 15-30 sec per retry

---

## Validation Checklist

Before declaring "MVP ready":

- [ ] All 6 nodes launch without errors
- [ ] `ros2 bag record` captures messages on all 6 topics
- [ ] `/camera/image_raw` has ~150+ messages (5 Hz × 30 sec)
- [ ] `/repair/toolpath` has ~100+ messages
- [ ] `/repair/executed` has 1-3 messages (only at end of repair)
- [ ] `/repair/quality_score` has ~60+ messages (2 Hz × 30 sec)
- [ ] State machine operates without hanging (timeouts work)
- [ ] Retry logic triggers correctly (modify quality_threshold to test)

---

## Next Session Checklist

```bash
# Fast re-entry into testing
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch + record in background
ros2 launch repairman_vision launch.py &
sleep 5
ros2 bag record -a -o session2_bag &
sleep 60
# Ctrl+C to stop both

# Analyze
ros2 bag info session2_bag
```

---

## Support Files

1. **README_REPAIRMAN.md** - Full architecture & deep dive
2. **QUICKSTART.md** - 5-minute getting started
3. **IMPLEMENTATION_SUMMARY.md** - What was built & why
4. **verify_setup.sh** - Automated verification
5. **launch/launch.py** - Node launcher config
6. This file (**BUILD_DEPLOY_COMMANDS.md**) - Command reference

---

**You're ready to deploy! 🚀**
