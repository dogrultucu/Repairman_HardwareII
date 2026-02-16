# Repairman Project - Next Chat Handoff

Date: 2026-02-16
Workspace: `/home/dogru/repairman_ws`

## 1) Current Priority
Ensure crack mask and generated toolpath match the real crack geometry.
This is critical before further robot realism work.

## 2) What Was Changed In This Session

### A) Crack-mask fidelity fixes
- `yolo_node` fallback mode was changed from synthetic curve generation to image-driven crack extraction.
- In no-YOLO mode, it now tries classical CV extraction (`blackhat + adaptive + Otsu + morphology + largest component`).
- Synthetic curve fallback is kept only as last-resort if no crack can be extracted.

Files:
- `src/repairman_vision/repairman_vision/yolo_node.py`
- `src/repairman_vision/launch/launch.py` (param `dummy_use_image_crack_extractor:=true`)

### B) Robot behavior realism (state-driven)
- Execution starts only when `/repair/state == REPAIR`.
- Robot motion is state-driven:
  - `SCAN/RESCAN`: scanning sweep
  - `DETECT/PLAN/EVALUATE/IDLE/PASS`: safe wait pose
  - `REPAIR`: toolpath tracking + extrusion
- Added minimum dwell times in state machine to make phase behavior visible.

Files:
- `src/repairman_vision/repairman_vision/execute_node.py`
- `src/repairman_vision/repairman_vision/arm_sim_node.py`
- `src/repairman_vision/repairman_vision/state_manager.py`
- `src/repairman_vision/launch/launch.py`

### C) Realtime pipeline visualizer
- Added a realtime visual node showing phase and percentage progress in RViz.

Files:
- `src/repairman_vision/repairman_vision/pipeline_visualizer_node.py`
- `src/repairman_vision/rviz/repairman_pipeline.rviz`
- `src/repairman_vision/setup.py`

## 3) How To Run (host WSL path)
```bash
cd ~/repairman_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select repairman_vision
source install/setup.bash

ros2 launch repairman_vision launch.py \
  use_moveit:=true \
  use_moveit_scene:=true \
  use_moveit_collision_check:=true \
  use_arm_sim:=true \
  require_repair_permission:=true \
  arm_state_driven_motion:=true \
  use_pipeline_visualizer:=true
```

## 4) Validation Checklist For Next Chat
1. Verify `/damage/mask` visually matches crack in `Camera` panel.
2. Verify `/repair/toolpath_overlay` centerline lies on crack center.
3. Verify robot only tracks in `REPAIR`; scan sweep only in `SCAN/RESCAN`.
4. Verify collision status topic updates:
   - `/repair/collision_status`
   - `/repair/in_collision`
5. If mismatch persists, tune fallback extractor thresholds in `yolo_node.py` and test on target crack images.

## 5) Important Note
If `ultralytics` is unavailable, pipeline uses fallback extraction.
For best quality and consistency, use a segmentation-capable YOLO model and publish true masks.
