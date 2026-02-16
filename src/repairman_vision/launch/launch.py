"""
Launch file for the Repairman Robot closed-loop repair pipeline.

Nodes launched:
  1. image_pub        - Fake camera, publishes /camera/image_raw
  2. yolo_node        - YOLO detector, publishes /damage/mask and /damage/annotated
  3. toolpath_node    - Mask to toolpath converter
  4. execute_node     - Simulated robot executor
  5. verify_node      - Quality scorer
  6. state_manager    - Closed-loop orchestrator
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_path, relative_path):
    yaml_path = os.path.join(package_path, relative_path)
    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory("repairman_vision")

    # Launch arguments with defaults
    image_path_arg = DeclareLaunchArgument(
        "image_path",
        default_value=os.path.join(os.path.expanduser("~"), "repairman_ws", "data", "test.png"),
        description="Path to test image for fake camera",
    )

    weights_path_arg = DeclareLaunchArgument(
        "weights_path",
        default_value=os.path.join(os.path.expanduser("~"), "repairman_ws", "weights", "best.pt"),
        description="Path to YOLO weights",
    )

    yolo_conf_arg = DeclareLaunchArgument(
        "yolo_conf",
        default_value="0.4",
        description="YOLO confidence threshold",
    )

    dummy_blackhat_kernel_arg = DeclareLaunchArgument(
        "dummy_blackhat_kernel",
        default_value="17",
        description="Fallback extractor blackhat kernel size (odd)",
    )

    dummy_adaptive_block_size_arg = DeclareLaunchArgument(
        "dummy_adaptive_block_size",
        default_value="31",
        description="Fallback extractor adaptive-threshold block size (odd)",
    )

    dummy_adaptive_c_arg = DeclareLaunchArgument(
        "dummy_adaptive_c",
        default_value="2.0",
        description="Fallback extractor adaptive-threshold C value",
    )

    dummy_open_kernel_arg = DeclareLaunchArgument(
        "dummy_open_kernel",
        default_value="3",
        description="Fallback extractor opening kernel size (odd)",
    )

    dummy_close_kernel_arg = DeclareLaunchArgument(
        "dummy_close_kernel",
        default_value="5",
        description="Fallback extractor closing kernel size (odd)",
    )

    dummy_min_component_area_arg = DeclareLaunchArgument(
        "dummy_min_component_area",
        default_value="80",
        description="Fallback extractor minimum component area in pixels",
    )

    dummy_keep_top_components_arg = DeclareLaunchArgument(
        "dummy_keep_top_components",
        default_value="1",
        description="Fallback extractor: number of top-scored components to keep",
    )

    dummy_component_score_mode_arg = DeclareLaunchArgument(
        "dummy_component_score_mode",
        default_value="perimeter_x_aspect",
        description='Fallback extractor component scoring mode: "perimeter_x_aspect" or "largest_area"',
    )

    workspace_width_arg = DeclareLaunchArgument(
        "workspace_width",
        default_value="1.0",
        description="Execution workspace width in meters",
    )

    workspace_height_arg = DeclareLaunchArgument(
        "workspace_height",
        default_value="1.0",
        description="Execution workspace height in meters",
    )

    workpiece_origin_x_arg = DeclareLaunchArgument(
        "workpiece_origin_x",
        default_value="0.72",
        description="Workpiece origin X inside workspace for mapped crack toolpath",
    )

    workpiece_origin_y_arg = DeclareLaunchArgument(
        "workpiece_origin_y",
        default_value="0.30",
        description="Workpiece origin Y inside workspace for mapped crack toolpath",
    )

    workpiece_width_arg = DeclareLaunchArgument(
        "workpiece_width",
        default_value="0.22",
        description="Workpiece width in workspace coordinates",
    )

    workpiece_height_arg = DeclareLaunchArgument(
        "workpiece_height",
        default_value="0.40",
        description="Workpiece height in workspace coordinates",
    )

    preferred_start_x_arg = DeclareLaunchArgument(
        "preferred_start_x",
        default_value="0.78",
        description="Preferred toolpath start X to reduce initial robot effort",
    )

    preferred_start_y_arg = DeclareLaunchArgument(
        "preferred_start_y",
        default_value="0.50",
        description="Preferred toolpath start Y to reduce initial robot effort",
    )

    toolpath_min_area_arg = DeclareLaunchArgument(
        "toolpath_min_area",
        default_value="0",
        description="Minimum selected component area for publishing toolpath (pixels)",
    )

    toolpath_mask_open_kernel_arg = DeclareLaunchArgument(
        "toolpath_mask_open_kernel",
        default_value="3",
        description="Toolpath pre-clean open kernel size (odd)",
    )

    toolpath_mask_close_kernel_arg = DeclareLaunchArgument(
        "toolpath_mask_close_kernel",
        default_value="3",
        description="Toolpath pre-clean close kernel size (odd)",
    )

    toolpath_component_select_mode_arg = DeclareLaunchArgument(
        "toolpath_component_select_mode",
        default_value="longest_skeleton",
        description='Toolpath component selector: "longest_skeleton" or "largest_area"',
    )

    toolpath_component_min_area_arg = DeclareLaunchArgument(
        "toolpath_component_min_area",
        default_value="80",
        description="Component-area filter before selector scoring (pixels)",
    )

    quality_threshold_arg = DeclareLaunchArgument(
        "quality_threshold",
        default_value="0.5",
        description="Quality score threshold for pass/fail",
    )

    scan_min_duration_arg = DeclareLaunchArgument(
        "scan_min_duration_sec",
        default_value="1.5",
        description="Minimum dwell time in SCAN state before DETECT",
    )

    detect_min_duration_arg = DeclareLaunchArgument(
        "detect_min_duration_sec",
        default_value="0.8",
        description="Minimum dwell time in DETECT state before PLAN",
    )

    plan_min_duration_arg = DeclareLaunchArgument(
        "plan_min_duration_sec",
        default_value="0.8",
        description="Minimum dwell time in PLAN state before REPAIR",
    )

    rescan_delay_arg = DeclareLaunchArgument(
        "rescan_delay_sec",
        default_value="2.0",
        description="Delay in RESCAN state before evaluation is allowed",
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Auto-start repair cycles",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with pipeline visualization config",
    )

    use_quality_plot_arg = DeclareLaunchArgument(
        "use_quality_plot",
        default_value="true",
        description="Start rqt_plot for /repair/quality_score visualization",
    )

    quality_plot_topic_arg = DeclareLaunchArgument(
        "quality_plot_topic",
        default_value="/repair/quality_score/data",
        description="Topic field plotted by rqt_plot",
    )

    simulate_extrusion_arg = DeclareLaunchArgument(
        "simulate_extrusion",
        default_value="true",
        description="Enable RViz extrusion marker simulation while executing toolpath",
    )

    require_repair_permission_arg = DeclareLaunchArgument(
        "require_repair_permission",
        default_value="true",
        description="Start execution only when state manager is in REPAIR state",
    )

    use_pipeline_visualizer_arg = DeclareLaunchArgument(
        "use_pipeline_visualizer",
        default_value="true",
        description="Show realtime phase + percentage progress marker overlay in RViz",
    )

    use_arm_sim_arg = DeclareLaunchArgument(
        "use_arm_sim",
        default_value="true",
        description="Drive a 6-axis demo robot model from executed toolpath",
    )

    arm_state_driven_motion_arg = DeclareLaunchArgument(
        "arm_state_driven_motion",
        default_value="true",
        description="Use /repair/state-driven scan/wait/repair behavior for arm simulation",
    )

    arm_tracking_alpha_arg = DeclareLaunchArgument(
        "arm_tracking_alpha",
        default_value="0.2",
        description="Arm IK tracking blend factor",
    )

    arm_target_smoothing_alpha_arg = DeclareLaunchArgument(
        "arm_target_smoothing_alpha",
        default_value="0.35",
        description="Smoothing on target path samples",
    )

    arm_yaw_smoothing_alpha_arg = DeclareLaunchArgument(
        "arm_yaw_smoothing_alpha",
        default_value="0.15",
        description="Smoothing factor for target yaw to reduce wrist jitter",
    )

    arm_max_joint_speed_arg = DeclareLaunchArgument(
        "max_joint_speed_rad_s",
        default_value="1.0",
        description="Max joint speed used by arm simulation",
    )

    arm_singularity_margin_arg = DeclareLaunchArgument(
        "singularity_margin",
        default_value="0.08",
        description="Singularity avoidance margin for IK branch",
    )

    arm_safe_min_radius_arg = DeclareLaunchArgument(
        "safe_min_radius",
        default_value="0.26",
        description="Min radial distance from base for collision/singularity avoidance",
    )

    arm_safe_max_radius_arg = DeclareLaunchArgument(
        "safe_max_radius",
        default_value="0.44",
        description="Max radial distance from base for stable IK",
    )

    arm_safe_work_z_arg = DeclareLaunchArgument(
        "safe_work_z",
        default_value="0.11",
        description="Minimum Z used by arm simulator to avoid table collisions",
    )

    arm_tool_tcp_offset_arg = DeclareLaunchArgument(
        "arm_tool_tcp_offset",
        default_value="0.23",
        description="TCP offset compensation from wrist to tool tip for path tracking",
    )

    arm_use_collision_guard_arg = DeclareLaunchArgument(
        "arm_use_collision_guard",
        default_value="true",
        description="If true, arm_sim uses /repair/in_collision guard behavior",
    )

    arm_retreat_on_collision_arg = DeclareLaunchArgument(
        "arm_retreat_on_collision",
        default_value="false",
        description="If true, arm_sim returns home when collision is detected",
    )

    arm_elbow_switch_penalty_arg = DeclareLaunchArgument(
        "arm_elbow_switch_penalty",
        default_value="0.25",
        description="Penalty for switching IK elbow branch to reduce unpredictable flips",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_dir, "rviz", "repairman_pipeline.rviz"),
        description="Path to RViz2 config file",
    )

    connect_world_to_repairman_arg = DeclareLaunchArgument(
        "connect_world_to_repairman",
        default_value="true",
        description="Publish static TF from robot world frame to repairman_map",
    )

    robot_world_frame_arg = DeclareLaunchArgument(
        "robot_world_frame",
        default_value="world",
        description="Root TF frame used by robot simulation (e.g. world or map)",
    )

    publish_demo_robot_arg = DeclareLaunchArgument(
        "publish_demo_robot",
        default_value="true",
        description="Publish a built-in demo URDF so RViz RobotModel is always visible",
    )

    use_moveit_arg = DeclareLaunchArgument(
        "use_moveit",
        default_value="false",
        description="Launch MoveIt move_group stack for planning and collision checking",
    )

    use_moveit_scene_arg = DeclareLaunchArgument(
        "use_moveit_scene",
        default_value="false",
        description="Publish environment collision objects to /planning_scene",
    )

    use_moveit_collision_check_arg = DeclareLaunchArgument(
        "use_moveit_collision_check",
        default_value="false",
        description="Check /joint_states with MoveIt /check_state_validity",
    )

    moveit_group_name_arg = DeclareLaunchArgument(
        "moveit_group_name",
        default_value="manipulator",
        description="MoveIt planning group used for state validity checks",
    )

    moveit_check_rate_arg = DeclareLaunchArgument(
        "moveit_check_rate_hz",
        default_value="5.0",
        description="Collision check query rate for /check_state_validity",
    )

    # Node 1: Image Publisher (fake camera)
    image_pub_node = Node(
        package="repairman_vision",
        executable="image_pub",
        name="image_pub",
        parameters=[
            {"image_path": LaunchConfiguration("image_path")},
            {"topic": "/camera/image_raw"},
            {"hz": 5.0},
        ],
        output="screen",
    )

    # Node 2: YOLO Detector
    yolo_node = Node(
        package="repairman_vision",
        executable="yolo_node",
        name="yolo_node",
        parameters=[
            {"weights_path": LaunchConfiguration("weights_path")},
            {"conf": LaunchConfiguration("yolo_conf")},
            {"imgsz": 640},
            {"camera_topic": "/camera/image_raw"},
            {"executed_topic": "/repair/executed"},
            {"mask_after_topic": "/damage/mask_after"},
            {"bbox_refine_crack": True},
            {"dummy_use_image_crack_extractor": True},
            {"dummy_blackhat_kernel": LaunchConfiguration("dummy_blackhat_kernel")},
            {"dummy_adaptive_block_size": LaunchConfiguration("dummy_adaptive_block_size")},
            {"dummy_adaptive_c": LaunchConfiguration("dummy_adaptive_c")},
            {"dummy_open_kernel": LaunchConfiguration("dummy_open_kernel")},
            {"dummy_close_kernel": LaunchConfiguration("dummy_close_kernel")},
            {"dummy_min_component_area": LaunchConfiguration("dummy_min_component_area")},
            {"dummy_keep_top_components": LaunchConfiguration("dummy_keep_top_components")},
            {"dummy_component_score_mode": LaunchConfiguration("dummy_component_score_mode")},
        ],
        output="screen",
    )

    # Node 3: Toolpath Generator
    toolpath_node = Node(
        package="repairman_vision",
        executable="toolpath_node",
        name="toolpath_node",
        parameters=[
            {"mask_topic": "/damage/mask"},
            {"toolpath_topic": "/repair/toolpath"},
            {"toolpath_path_topic": "/repair/toolpath_path"},
            {"toolpath_overlay_topic": "/repair/toolpath_overlay"},
            {"frame_id": "repairman_map"},
            {"workspace_width": LaunchConfiguration("workspace_width")},
            {"workspace_height": LaunchConfiguration("workspace_height")},
            {"output_in_workspace": True},
            {"workpiece_origin_x": LaunchConfiguration("workpiece_origin_x")},
            {"workpiece_origin_y": LaunchConfiguration("workpiece_origin_y")},
            {"workpiece_width": LaunchConfiguration("workpiece_width")},
            {"workpiece_height": LaunchConfiguration("workpiece_height")},
            {"path_smooth_window": 9},
            {"project_to_safe_annulus": True},
            {"safe_base_x": 0.5},
            {"safe_base_y": 0.5},
            {"safe_min_radius": LaunchConfiguration("safe_min_radius")},
            {"safe_max_radius": LaunchConfiguration("safe_max_radius")},
            {"reverse_for_easy_start": True},
            {"preferred_start_x": LaunchConfiguration("preferred_start_x")},
            {"preferred_start_y": LaunchConfiguration("preferred_start_y")},
            {"min_area": LaunchConfiguration("toolpath_min_area")},
            {"mask_open_kernel": LaunchConfiguration("toolpath_mask_open_kernel")},
            {"mask_close_kernel": LaunchConfiguration("toolpath_mask_close_kernel")},
            {"component_select_mode": LaunchConfiguration("toolpath_component_select_mode")},
            {"component_min_area": LaunchConfiguration("toolpath_component_min_area")},
        ],
        output="screen",
    )

    # Node 4: Execution Simulator
    execute_node = Node(
        package="repairman_vision",
        executable="execute_node",
        name="execute_node",
        parameters=[
            {"toolpath_topic": "/repair/toolpath"},
            {"state_topic": "/repair/state"},
            {"executed_topic": "/repair/executed"},
            {"executed_path_topic": "/repair/executed_path"},
            {"require_repair_state": LaunchConfiguration("require_repair_permission")},
            {"execute_rate_hz": 10.0},
            {"workspace_width": LaunchConfiguration("workspace_width")},
            {"workspace_height": LaunchConfiguration("workspace_height")},
            {"path_frame": "repairman_map"},
            {"accept_updates_while_executing": False},
            {"path_publish_stride": 1},
            {"publish_progress_path": True},
            {"simulate_extrusion": LaunchConfiguration("simulate_extrusion")},
            {"extrusion_markers_topic": "/repair/extrusion_markers"},
            {"extrusion_line_width": 0.02},
            {"extrusion_height": 0.01},
            {"toolhead_size": 0.04},
        ],
        output="screen",
    )

    # Node 5: Verification (Quality Scorer)
    verify_node = Node(
        package="repairman_vision",
        executable="verify_node",
        name="verify_node",
        parameters=[
            {"mask_before_topic": "/damage/mask"},
            {"mask_after_topic": "/damage/mask_after"},
            {"quality_topic": "/repair/quality_score"},
            {"executed_topic": "/repair/executed"},
            {"compute_rate_hz": 2.0},
            {"use_latest_as_after": False},
        ],
        output="screen",
    )

    # Node 6: State Manager (Orchestrator)
    state_manager_node = Node(
        package="repairman_vision",
        executable="state_manager",
        name="state_manager",
        parameters=[
            {"cycle_period_sec": 30.0},
            {"quality_threshold": LaunchConfiguration("quality_threshold")},
            {"rescan_delay_sec": LaunchConfiguration("rescan_delay_sec")},
            {"scan_min_duration_sec": LaunchConfiguration("scan_min_duration_sec")},
            {"detect_min_duration_sec": LaunchConfiguration("detect_min_duration_sec")},
            {"plan_min_duration_sec": LaunchConfiguration("plan_min_duration_sec")},
            {"max_repair_passes": 2},
            {"auto_start": LaunchConfiguration("auto_start")},
            {"verbose": True},
        ],
        output="screen",
    )

    # Static TF to make RViz fixed frame stable for /repair/executed_path visualization
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="repairman_static_tf",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "repairman_map",
            "--child-frame-id", "camera",
        ],
        output="screen",
    )

    # Optional bridge between the robot simulation TF tree and repairman_map.
    world_to_repairman_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="repairman_world_bridge_tf",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", LaunchConfiguration("robot_world_frame"),
            "--child-frame-id", "repairman_map",
        ],
        condition=IfCondition(LaunchConfiguration("connect_world_to_repairman")),
        output="screen",
    )

    demo_urdf_path = os.path.join(pkg_dir, "urdf", "repairman_demo.urdf")
    with open(demo_urdf_path, "r", encoding="utf-8") as urdf_file:
        demo_robot_description = urdf_file.read()

    demo_srdf_path = os.path.join(pkg_dir, "moveit", "repairman_demo.srdf")
    try:
        with open(demo_srdf_path, "r", encoding="utf-8") as srdf_file:
            demo_robot_semantic = srdf_file.read()
    except OSError:
        demo_robot_semantic = ""

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(pkg_dir, os.path.join("moveit", "kinematics.yaml"))
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(pkg_dir, os.path.join("moveit", "joint_limits.yaml"))
    }
    ompl_planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planning_request_adapters/AddTimeOptimalParameterization "
                "default_planning_request_adapters/FixWorkspaceBounds "
                "default_planning_request_adapters/FixStartStateBounds "
                "default_planning_request_adapters/FixStartStateCollision "
                "default_planning_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_pipeline["ompl"].update(
        load_yaml(pkg_dir, os.path.join("moveit", "ompl_planning.yaml"))
    )
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    demo_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="repairman_demo_state_publisher",
        parameters=[{"robot_description": demo_robot_description}],
        condition=IfCondition(LaunchConfiguration("publish_demo_robot")),
        output="screen",
    )

    demo_robot_description_topic_node = Node(
        package="repairman_vision",
        executable="robot_description_pub",
        name="repairman_robot_description_pub",
        parameters=[
            {"topic": "/robot_description"},
            {"robot_description": demo_robot_description},
            {"publish_rate_hz": 0.5},
        ],
        condition=IfCondition(LaunchConfiguration("publish_demo_robot")),
        output="screen",
    )

    arm_sim_node = Node(
        package="repairman_vision",
        executable="arm_sim_node",
        name="arm_sim_node",
        parameters=[
            {"path_topic": "/repair/executed_path"},
            {"executed_topic": "/repair/executed"},
            {"state_topic": "/repair/state"},
            {"joint_state_topic": "/joint_states"},
            {"publish_rate_hz": 30.0},
            {"use_state_driven_motion": LaunchConfiguration("arm_state_driven_motion")},
            {"path_stale_timeout_sec": 1.0},
            {"scan_cycle_sec": 4.0},
            {"scan_sweep_joint1_deg": 22.0},
            {"scan_sweep_joint4_deg": 16.0},
            {"wait_pose": [0.35, -1.20, 1.85, 0.0, -0.65, 0.0]},
            {"scan_pose": [0.20, -1.05, 1.70, 0.0, -0.80, 0.0]},
            {"tracking_alpha": LaunchConfiguration("arm_tracking_alpha")},
            {"target_smoothing_alpha": LaunchConfiguration("arm_target_smoothing_alpha")},
            {"yaw_smoothing_alpha": LaunchConfiguration("arm_yaw_smoothing_alpha")},
            {"max_joint_speed_rad_s": LaunchConfiguration("max_joint_speed_rad_s")},
            {"singularity_margin": LaunchConfiguration("singularity_margin")},
            {"safe_min_radius": LaunchConfiguration("safe_min_radius")},
            {"safe_max_radius": LaunchConfiguration("safe_max_radius")},
            {"safe_work_z": LaunchConfiguration("safe_work_z")},
            {"tool_tcp_offset": LaunchConfiguration("arm_tool_tcp_offset")},
            {"collision_topic": "/repair/in_collision"},
            {"use_collision_guard": LaunchConfiguration("arm_use_collision_guard")},
            {"retreat_on_collision": LaunchConfiguration("arm_retreat_on_collision")},
            {"elbow_branch_switch_penalty": LaunchConfiguration("arm_elbow_switch_penalty")},
            {"base_x": 0.5},
            {"base_y": 0.5},
            {"shoulder_z": 0.17},
            {"work_z": 0.01},
            {"link_2": 0.28},
            {"link_3": 0.24},
        ],
        condition=IfCondition(LaunchConfiguration("use_arm_sim")),
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        parameters=[
            {"robot_description": demo_robot_description},
            {"robot_description_semantic": demo_robot_semantic},
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline,
            planning_scene_monitor_parameters,
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": False},
            {"capabilities": ""},
            {"disable_capabilities": "move_group/ExecuteTrajectoryAction"},
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit")),
        output="screen",
    )

    planning_scene_node = Node(
        package="repairman_vision",
        executable="planning_scene_node",
        name="planning_scene_node",
        parameters=[
            {"planning_scene_topic": "/planning_scene"},
            {"frame_id": "repairman_map"},
            {"publish_rate_hz": 0.5},
            {"table_size": [1.2, 1.2, 0.08]},
            {"table_pose": [0.5, 0.5, -0.06]},
            {"workpiece_size": [LaunchConfiguration("workpiece_width"), LaunchConfiguration("workpiece_height"), 0.04]},
            {
                "workpiece_pose": [
                    PythonExpression(
                        [
                            "float(",
                            LaunchConfiguration("workpiece_origin_x"),
                            ") + float(",
                            LaunchConfiguration("workpiece_width"),
                            ")/2.0",
                        ]
                    ),
                    PythonExpression(
                        [
                            "float(",
                            LaunchConfiguration("workpiece_origin_y"),
                            ") + float(",
                            LaunchConfiguration("workpiece_height"),
                            ")/2.0",
                        ]
                    ),
                    0.03,
                ]
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit_scene")),
        output="screen",
    )

    collision_check_node = Node(
        package="repairman_vision",
        executable="collision_check_node",
        name="collision_check_node",
        parameters=[
            {"joint_state_topic": "/joint_states"},
            {"collision_topic": "/repair/in_collision"},
            {"status_topic": "/repair/collision_status"},
            {"marker_topic": "/repair/collision_marker"},
            {"state_validity_service": "/check_state_validity"},
            {"group_name": LaunchConfiguration("moveit_group_name")},
            {"check_rate_hz": LaunchConfiguration("moveit_check_rate_hz")},
            {"marker_frame": "repairman_map"},
            {"marker_pose": [0.15, 0.15, 0.30]},
            {"marker_scale": 0.08},
        ],
        condition=IfCondition(LaunchConfiguration("use_moveit_collision_check")),
        output="screen",
    )

    pipeline_visualizer_node = Node(
        package="repairman_vision",
        executable="pipeline_visualizer_node",
        name="pipeline_visualizer_node",
        parameters=[
            {"state_topic": "/repair/state"},
            {"marker_topic": "/repair/pipeline_markers"},
            {"status_topic": "/repair/pipeline_status"},
            {"stage_progress_topic": "/repair/stage_progress"},
            {"cycle_progress_topic": "/repair/cycle_progress"},
            {"publish_rate_hz": 5.0},
            {"frame_id": "repairman_map"},
            {"anchor_x": 0.15},
            {"anchor_y": 0.88},
            {"anchor_z": 0.38},
            {"bar_width": 0.70},
            {"bar_height": 0.02},
            {"stage_dot_size": 0.03},
            {"text_size": 0.06},
        ],
        condition=IfCondition(LaunchConfiguration("use_pipeline_visualizer")),
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="repairman_rviz",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    quality_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="repairman_quality_plot",
        arguments=[LaunchConfiguration("quality_plot_topic")],
        condition=IfCondition(LaunchConfiguration("use_quality_plot")),
        output="screen",
    )

    return LaunchDescription(
        [
            image_path_arg,
            weights_path_arg,
            yolo_conf_arg,
            dummy_blackhat_kernel_arg,
            dummy_adaptive_block_size_arg,
            dummy_adaptive_c_arg,
            dummy_open_kernel_arg,
            dummy_close_kernel_arg,
            dummy_min_component_area_arg,
            dummy_keep_top_components_arg,
            dummy_component_score_mode_arg,
            workspace_width_arg,
            workspace_height_arg,
            workpiece_origin_x_arg,
            workpiece_origin_y_arg,
            workpiece_width_arg,
            workpiece_height_arg,
            preferred_start_x_arg,
            preferred_start_y_arg,
            toolpath_min_area_arg,
            toolpath_mask_open_kernel_arg,
            toolpath_mask_close_kernel_arg,
            toolpath_component_select_mode_arg,
            toolpath_component_min_area_arg,
            quality_threshold_arg,
            scan_min_duration_arg,
            detect_min_duration_arg,
            plan_min_duration_arg,
            rescan_delay_arg,
            auto_start_arg,
            use_rviz_arg,
            use_quality_plot_arg,
            quality_plot_topic_arg,
            simulate_extrusion_arg,
            require_repair_permission_arg,
            use_pipeline_visualizer_arg,
            use_arm_sim_arg,
            arm_state_driven_motion_arg,
            arm_tracking_alpha_arg,
            arm_target_smoothing_alpha_arg,
            arm_yaw_smoothing_alpha_arg,
            arm_max_joint_speed_arg,
            arm_singularity_margin_arg,
            arm_safe_min_radius_arg,
            arm_safe_max_radius_arg,
            arm_safe_work_z_arg,
            arm_tool_tcp_offset_arg,
            arm_use_collision_guard_arg,
            arm_retreat_on_collision_arg,
            arm_elbow_switch_penalty_arg,
            rviz_config_arg,
            connect_world_to_repairman_arg,
            robot_world_frame_arg,
            publish_demo_robot_arg,
            use_moveit_arg,
            use_moveit_scene_arg,
            use_moveit_collision_check_arg,
            moveit_group_name_arg,
            moveit_check_rate_arg,
            image_pub_node,
            yolo_node,
            toolpath_node,
            execute_node,
            verify_node,
            state_manager_node,
            static_tf_node,
            world_to_repairman_tf_node,
            demo_robot_state_publisher_node,
            demo_robot_description_topic_node,
            arm_sim_node,
            move_group_node,
            planning_scene_node,
            collision_check_node,
            pipeline_visualizer_node,
            rviz_node,
            quality_plot_node,
        ]
    )
