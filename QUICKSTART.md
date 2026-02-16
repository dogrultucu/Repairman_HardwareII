# QUICK START: Repairman Robot Pipeline

## In 5 Minutes

### 1. Prepare Environment
```bash
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 2. Launch Everything
```bash
ros2 launch repairman_vision launch.py
```

You should see 6 nodes starting:
- `image_pub` (fake camera)
- `yolo_node` (detection)
- `toolpath_node` (path generator)
- `execute_node` (simulator)
- `verify_node` (quality scorer)
- `state_manager` (orchestrator)

### 3. Validate in Another Terminal
```bash
# Keep original launch running, open new terminal

cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Record topics (daemon-free validation)
ros2 bag record -a -o full_pipeline_bag
```

Let it run for ~30-60 seconds, then Ctrl+C.

### 4. Check Results
```bash
ros2 bag info full_pipeline_bag

# You should see:
# - /camera/image_raw: ~150-300 msgs
# - /damage/mask: ~150-300 msgs
# - /damage/annotated: ~150-300 msgs
# - /repair/toolpath: ~150-300 msgs
# - /repair/executed: 1-3 msgs (at end of repair)
# - /repair/quality_score: ~60+ msgs
```

✅ If you see messages on all topics → **Pipeline works!**

---

## Detailed Instructions

See [README_REPAIRMAN.md](README_REPAIRMAN.md) for:
- Full architecture and node descriptions
- Parameter tuning
- Troubleshooting
- Hardware integration guide
- Debug strategies using `ros2 bag`

---

## Custom Launch Parameters

```bash
# Change quality threshold
ros2 launch repairman_vision launch.py quality_threshold:=0.6

# Use different test image
ros2 launch repairman_vision launch.py image_path:=/path/to/my/image.png

# Disable auto-start (manual cycles)
ros2 launch repairman_vision launch.py auto_start:=false
```

---

## Troubleshooting

**"Command 'ros2' not found"**
→ Run: `source /opt/ros/jazzy/setup.bash`

**"YOLO weights not found"**
→ Ensure: `~/repairman_ws/weights/best.pt` exists

**"Topic echo times out"**
→ Use `ros2 bag record` instead (daemon-free)

**Build fails**
→ Run: `colcon build --symlink-install --packages-select repairman_vision`

See [README_REPAIRMAN.md](README_REPAIRMAN.md) **Troubleshooting** section for more.
