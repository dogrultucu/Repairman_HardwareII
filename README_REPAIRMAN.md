# Repairman Robot: Closed-Loop Repair Pipeline

## Overview

**Repairman Robot** is a perception-driven closed-loop repair pipeline for ROS 2 Jazzy. It scans a surface with an RGB camera, detects damage (cracks / surface deformations) using a trained YOLO model, generates a repair toolpath, executes a simulated repair action, re-scans to verify quality, and decides if a second pass is needed.

### Pipeline Flow
```
SCAN → DETECT (YOLO) → PLAN (Toolpath) → REPAIR (Execute) → RESCAN → EVALUATE (Quality) → PASS / REPAIR_AGAIN
```

---

## Project Structure

```
~/repairman_ws/
├── src/repairman_vision/
│   ├── package.xml                           # ROS 2 package manifest
│   ├── setup.py                              # Python package setup
│   ├── setup.cfg
│   ├── repairman_vision/
│   │   ├── __init__.py
│   │   ├── image_pub.py                      # Fake camera node
│   │   ├── yolo_node.py                      # YOLO detection node
│   │   ├── toolpath_node.py                  # Mask → Toolpath converter
│   │   ├── execute_node.py                   # ⭐ NEW: Execution simulator
│   │   ├── verify_node.py                    # ⭐ NEW: Quality scorer
│   │   └── state_manager.py                  # ⭐ NEW: Closed-loop orchestrator
│   ├── launch/
│   │   └── launch.py                         # ⭐ NEW: Launch all nodes
│   └── resource/
├── data/
│   └── test.png                               # Test image for fake camera
├── weights/
│   └── best.pt                                # YOLO model weights
└── docker-compose.yml                         # Docker setup (optional)
```

---

## Dependencies

### System Requirements
- **OS**: Ubuntu (tested on Ubuntu 22.04)
- **ROS 2**: Jazzy
- **Python**: 3.10+

### Python Packages

The following are auto-installed by `colcon`:

- `rclpy` – ROS 2 Python client library
- `sensor_msgs` – Image message type
- `geometry_msgs` – Polygon message type
- `nav_msgs` – Path message type
- `std_msgs` – Basic message types
- `cv_bridge` – OpenCV ↔ ROS Image conversion
- `opencv-python` – Image processing
- `ultralytics` – YOLO model (for detection)

### Install Additional Dependencies (if needed)

```bash
# Inside ROS 2 environment
pip install ultralytics opencv-python
```

---

## Build Instructions

### 1. Navigate to Workspace

```bash
cd ~/repairman_ws
```

### 2. Source ROS 2 Setup (Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
```

### 3. Build the Package

```bash
colcon build --symlink-install --packages-select repairman_vision
```

**Note**: `--symlink-install` allows editing Python files without rebuilding.

### 4. Source the Workspace

```bash
source install/setup.bash
```

---

## Run Instructions

### Option A: Launch All Nodes at Once (Recommended)

```bash
# Terminal 1: Start the entire pipeline
ros2 launch repairman_vision launch.py
```

**Launch Arguments** (optional):

```bash
# Custom image path
ros2 launch repairman_vision launch.py image_path:=~/custom_image.png

# Custom quality threshold (0.0-1.0)
ros2 launch repairman_vision launch.py quality_threshold:=0.6

# Disable auto-start (manual cycle triggering)
ros2 launch repairman_vision launch.py auto_start:=false
```

---

### Option B: Manual Terminal-by-Terminal (for debugging)

**Terminal 1: Image Publisher (Fake Camera)**
```bash
ros2 run repairman_vision image_pub --ros-args \
  -p image_path:=~/repairman_ws/data/test.png \
  -p topic:=/camera/image_raw \
  -p hz:=5.0
```

**Terminal 2: YOLO Detector**
```bash
# Activate Python env with ultralytics (if using venv)
python -m repairman_vision.yolo_node
```
Or plain:
```bash
ros2 run repairman_vision yolo_node --ros-args \
  -p weights_path:=~/repairman_ws/weights/best.pt \
  -p conf:=0.4 \
  -p imgsz:=640
```

**Terminal 3: Toolpath Generator**
```bash
ros2 run repairman_vision toolpath_node --ros-args -p min_area:=0
```

**Terminal 4: Executor**
```bash
ros2 run repairman_vision execute_node
```

**Terminal 5: Verifier (Quality Scorer)**
```bash
ros2 run repairman_vision verify_node --ros-args \
  -p use_latest_as_after:=false
```

**Terminal 6: State Manager (Orchestrator)**
```bash
ros2 run repairman_vision state_manager --ros-args \
  -p auto_start:=true \
  -p quality_threshold:=0.5
```

---

## Validation & Testing (Daemon-Free)

Because ROS 2 CLI daemon can be unstable in some environments, we use `ros2 bag record` for robust proof of message publication.

### Record Key Topics

```bash
# Terminal 7: Record all topics (safe approach)
ros2 bag record -a -o all_topics_bag

# Or record specific topics only
ros2 bag record \
  /camera/image_raw \
  /damage/mask \
  /damage/annotated \
  /repair/toolpath \
  /repair/executed \
  /repair/quality_score \
  -o repair_pipeline_bag
```

### Inspect Recorded Bag

```bash
# View bag summary
ros2 bag info repair_pipeline_bag

# Expected output:
# Files:             repair_pipeline_bag_0.mcap
# Bag size:          X MiB
# Messages:          N
# Duration:          T seconds
# Topic: /camera/image_raw | Count: ~N
# Topic: /damage/mask | Count: ~N
# Topic: /damage/annotated | Count: ~N
# Topic: /repair/toolpath | Count: ~N
# Topic: /repair/executed | Count: ~1 (at end of execution)
# Topic: /repair/quality_score | Count: ~M (periodic)
```

### Playback and Verify

```bash
# Playback bag (in separate terminal)
ros2 bag play repair_pipeline_bag --loop

# In another terminal, monitor topics
ros2 topic echo /repair/toolpath

# Or use RViz for visualization
rviz2 -d ~/repairman_ws/rviz_config.rviz  # (optional config file)
```

### Message Count Sanity Checks

After running the full pipeline for ~30s:

| Topic | Expected Count | Notes |
|-------|---|---|
| `/camera/image_raw` | ~150 | 5 Hz × 30 s |
| `/damage/mask` | ~150 | 1:1 with image |
| `/damage/annotated` | ~150 | YOLO output |
| `/repair/toolpath` | ~150 | 1:1 with mask (or fewer if filtered) |
| `/repair/executed` | 1–3 | Only after repair pass |
| `/repair/quality_score` | ~60 | 2 Hz (from verify_node) |

---

## Debug Strategies

### 1. Enable Verbose Logging

```bash
# In any terminal running a node
export ROS_LOG_LEVEL=DEBUG
ros2 run repairman_vision state_manager --ros-args -p verbose:=true
```

### 2. Monitor Individual Topics (Quick Check)

```bash
# In a fresh terminal, attempt to echo a topic
# (may timeout with daemon issues, so use bag record instead)
ros2 topic echo /repair/toolpath --once
```

### 3. Check Node Status

```bash
# List all running nodes
ros2 node list

# Show publishers/subscribers for a node
ros2 node info /execute_node
```

### 4. View Message Types

```bash
# Inspect message structure
ros2 interface show geometry_msgs/msg/Polygon
ros2 interface show std_msgs/msg/Float32
```

### 5. Replay Recorded Bag to Test Pipeline

```bash
# Save previous bag
cp toolpath_bag toolpath_bag_reference

# Record new run and compare
ros2 bag record /repair/toolpath -o new_run_bag
ros2 bag info new_run_bag
```

---

## State Machine Overview

The `state_manager` node orchestrates the repair pipeline:

### States

```
IDLE
  ↓ (auto-trigger or external signal)
SCAN
  ↓ (image received)
DETECT
  ↓ (mask received)
PLAN
  ↓ (toolpath received)
REPAIR
  ↓ (execution done)
RESCAN
  ↓ (wait + re-scan image)
EVALUATE
  ↓ (quality score computed)
  ├─→ PASS (if quality ≥ threshold) → IDLE
  └─→ REPAIR_AGAIN (if quality < threshold & pass_count < max) → REPAIR
```

### Configuration Parameters

- `cycle_period_sec`: Time between auto-triggered cycles (default: 30 s)
- `quality_threshold`: Quality score threshold for pass (default: 0.5)
- `rescan_delay_sec`: Wait time after repair before re-scanning (default: 2 s)
- `max_repair_passes`: Max consecutive repair attempts (default: 2)
- `auto_start`: Auto-trigger cycles (default: true)
- `verbose`: Detailed logging (default: true)

---

## Node Reference

### 1. `image_pub` (Fake Camera)

Publishes a static image repeatedly.

**Parameters**:
- `image_path`: Path to test image
- `topic`: Output topic (default: `/camera/image_raw`)
- `hz`: Publication rate (default: 5.0)

**Output**: `Image (sensor_msgs/msg/Image)` on `/camera/image_raw`

---

### 2. `yolo_node` (YOLO Detector)

Runs YOLO inference on incoming images and generates a binary mask.

**Parameters**:
- `weights_path`: Path to `best.pt`
- `conf`: Confidence threshold (default: 0.4)
- `imgsz`: YOLO input size (default: 640)
- `camera_topic`: Input image topic (default: `/camera/image_raw`)

**Inputs**: `Image` on `/camera/image_raw`

**Outputs**:
- `Image (mono8)` on `/damage/mask`
- `Image (bgr8)` on `/damage/annotated`

---

### 3. `toolpath_node` (Mask → Toolpath)

Converts a damage mask into a repair toolpath (polygon).

**Parameters**:
- `mask_topic`: Input mask topic (default: `/damage/mask`)
- `toolpath_topic`: Output toolpath topic (default: `/repair/toolpath`)
- `min_area`: Minimum damage area (px²) to process (default: 300)

**Inputs**: `Image (mono8)` on `/damage/mask`

**Outputs**: `Polygon (geometry_msgs/msg/Polygon)` on `/repair/toolpath`

---

### 4. `execute_node` (Execution Simulator) ⭐ NEW

Simulates robot execution of a toolpath.

**Parameters**:
- `toolpath_topic`: Input toolpath topic (default: `/repair/toolpath`)
- `executed_topic`: Output execution signal (default: `/repair/executed`)
- `executed_path_topic`: Output path for visualization (default: `/repair/executed_path`)
- `execute_rate_hz`: Step rate during execution (default: 10 Hz)
- `workspace_width`, `workspace_height`: Workspace bounds for normalization

**Inputs**: `Polygon` on `/repair/toolpath`

**Outputs**:
- `Bool` on `/repair/executed` (True when done)
- `Path (nav_msgs/msg/Path)` on `/repair/executed_path` (for visualization)

---

### 5. `verify_node` (Quality Scorer) ⭐ NEW

Computes repair quality by comparing before/after masks.

**Parameters**:
- `mask_before_topic`: Pre-repair mask (default: `/damage/mask`)
- `mask_after_topic`: Post-repair mask (default: `/damage/mask_after`)
- `quality_topic`: Output quality score (default: `/repair/quality_score`)
- `compute_rate_hz`: Scoring rate (default: 2 Hz)
- `use_latest_as_after`: If true, use latest `/damage/mask` as "after" (for MVP)

**Inputs**:
- `Image (mono8)` on `/damage/mask` (before)
- `Image (mono8)` on `/damage/mask_after` (after)

**Outputs**: `Float32` on `/repair/quality_score` (range 0.0–1.0)

**Quality Formula**:
```
quality = clamp((before_area - after_area) / max(before_area, eps), 0, 1)
```
where `before_area` and `after_area` are non-zero pixel counts.

---

### 6. `state_manager` (Orchestrator) ⭐ NEW

Closed-loop state machine orchestrator.

**Parameters**: See "State Machine Overview" above.

**Inputs**:
- `Image` on `/camera/image_raw`
- `Image` on `/damage/mask`
- `Polygon` on `/repair/toolpath`
- `Bool` on `/repair/executed`
- `Float32` on `/repair/quality_score`

**Outputs**:
- `Bool` on `/repair/start_cycle` (external trigger)
- `String` on `/repair/state` (current state name, diagnostics)

---

## MVP Scope & Known Limitations

### What Works Now

✅ Single-pass SCAN → DETECT → PLAN → REPAIR → RESCAN → EVALUATE pipeline  
✅ Toolpath generation from YOLO mask proven to work (258 messages recorded)  
✅ Quality scoring based on mask area reduction  
✅ Simulated robot execution (point-by-point iteration)  
✅ State machine with retry logic (up to `max_repair_passes`)  
✅ Daemon-free validation using `ros2 bag record`  

### Limitations (For Future Sprints)

- **Verify Node**: Currently uses simulated "after" image. Real implementation requires:
  - Second camera capture after material deposition (time-controlled)
  - Or YOLO re-inference after wait period
  
- **Execute Node**: Does not interface with real robot controller. To integrate:
  - Replace point iteration with actual joint/extruder commands
  - Use `ros2 action` or `ros2 service` for trajectory execution
  - Connect extruder PWM or analog output for material deposition

- **Visualizer**: No RViz config provided yet (optional for this sprint)

- **Error Handling**: Minimal error recovery (timeouts reset to IDLE)

---

## Hardware Integration Notes

### For Real Robot Integration

When connecting to actual hardware:

#### 1. **Camera Interface**

**Swap**: Fake `image_pub` → Real camera driver  
**Options**:
- `usb_cam` ROS 2 package (standard USB camera)
- `realsense-ros` for Intel RealSense
- `zed-ros2-wrapper` for Stereolabs ZED

```bash
# Example: usb_cam instead of image_pub
ros2 run usb_cam usb_cam_node_exe --ros-args -p camera_info_url:=file://path/to/calibration.yaml
```

Ensure output topic is remapped to `/camera/image_raw`.

#### 2. **Robot Arm / Manipulator**

**Swap**: `execute_node` → Real robot controller  
**Interface**:
- Use `ros2 action` (`FollowJointTrajectory` from `trajectory_msgs`)
- Or `ros2 service` for low-level motion commands
- Or direct publisher to `/joint_commands` (hardware-dependent)

**Minimal Action Server Example**:
```python
# In execute_node replacement
self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
# Convert polygon points to joint angles, send goal
```

#### 3. **Extruder / Deposition System**

**Control Interface**:
- PWM signal (GPIO, USB relay, or DAC)
- Analog command (0–10V)
- Serial protocol to extruder controller

**Topic to Command Flow**:
```
/repair/toolpath → execute_node reads point rate → 
  publishes /extruder/on (std_msgs/Bool) or 
  /extruder/command (std_msgs/UInt16 for PWM %)
```

#### 4. **Verification Camera**

**Option A**: Same camera as scan (simplest)
- Use timestamp + delay to trigger re-scan

**Option B**: Separate verification camera
- Publish to `/damage/mask_after` explicitly
- Mounted for top-down surface inspection

```python
# In verify_node replacement
# Listen for robot completion signal, trigger secondary image capture
self.sub_executed.create_subscription(Bool, '/repair/executed', self.trigger_verification)
```

#### 5. **Closed-Loop Refinements for Hardware**

- **Calibration**: Map pixel coordinates (mask) → robot workspace (XYZ or joint angles)
- **Tolerance Band**: Quality score may plateau; set reasonable threshold (e.g., 0.6–0.7)
- **Second Pass Strategy**: Adjust deposition speed, amount, or pressure instead of repeating identical path
- **Timeout Tuning**: Increase `state_timeout` for slow robot movements

---

## Troubleshooting

### Issue: "DDS initialization failed" or Topic Echo Timeouts

**Root Cause**: ROS 2 CLI daemon instability (known in this environment).

**Solution**:
1. Use `ros2 bag record` for validation instead of `ros2 topic echo`
2. Verify nodes are running: `ros2 node list`
3. Rebuild with explicit DDS config:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   colcon build --symlink-install
   source install/setup.bash
   ```

### Issue: YOLO Node Crashes or Hangs

**Root Cause**: Missing ultralytics or weights file.

**Solution**:
```bash
pip install ultralytics
# Verify weights are at ~/repairman_ws/weights/best.pt
ls -la ~/repairman_ws/weights/best.pt
```

### Issue: State Manager Stays in One State

**Root Cause**: No message arriving on expected topic (use state timeouts).

**Solution**:
1. Check node is publishing:
   ```bash
   ros2 node list | grep yolo_node  # etc.
   ```
2. Enable debug logging:
   ```bash
   export ROS_LOG_LEVEL=DEBUG
   ros2 run repairman_vision state_manager --ros-args -p verbose:=true
   ```
3. Record bag and analyze:
   ```bash
   ros2 bag record -a
   ros2 bag info <bag_name>
   ```

### Issue: Quality Score Always 0.0 or 1.0

**Root Cause**: Verify node expecting `/damage/mask_after` but only `/damage/mask` published.

**Solution**:
Enable MVP mode (use latest mask):
```bash
ros2 run repairman_vision verify_node --ros-args -p use_latest_as_after:=true
```

Or remap topic:
```bash
ros2 run repairman_vision verify_node --ros-args -r /damage/mask_after:=/damage/mask
```

### Issue: Bag File Very Large or Unresponsive

**Root Cause**: Recording all image topics (high bandwidth).

**Solution**:
```bash
# Record only non-image topics (much smaller)
ros2 bag record \
  /repair/toolpath \
  /repair/executed \
  /repair/quality_score \
  -o compact_bag
```

---

## Clean Build & Reset

If you experience persistent issues:

```bash
# Clean build artifacts
cd ~/repairman_ws
rm -rf build install log

# Rebuild from scratch
colcon build --symlink-install --packages-select repairman_vision

# Source anew
source install/setup.bash

# Test single node
ros2 run repairman_vision image_pub
```

---

## Next Steps / Future Work

1. **Real Hardware Integration**
   - Replace `image_pub` with real camera
   - Replace `execute_node` with actual robot controller
   - Test end-to-end on real surface

2. **Advanced Quality Metrics**
   - Depth-based erosion measurement
   - Surface smoothness (contrast analysis)
   - Coverage verification (% defect covered)

3. **Learning & Optimization**
   - Adaptive parameters (tool speed, deposition rate) based on quality feedback
   - Second-pass path offsets (spiral inward, offset toolpath)

4. **Error Recovery**
   - Detect if robot is stuck (no movement), trigger recovery
   - Fallback paths if toolpath unreachable

5. **Multi-Defect Handling**
   - Sort toolpaths by priority, execute largest first
   - Parallel multi-robot coordination (future)

---

## Authors & References

- **Project**: Repairman Robot (ROS 2 Jazzy MVP)
- **Framework**: ROS 2 Jazzy, rclpy, OpenCV, YOLO (ultralytics)
- **Created**: Feb 2026

---

## License

MIT

---

**Questions or Issues?** Check the troubleshooting section or file logs from `ros2 bag record`.
