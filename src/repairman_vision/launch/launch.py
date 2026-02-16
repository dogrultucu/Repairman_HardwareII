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
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


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

    quality_threshold_arg = DeclareLaunchArgument(
        "quality_threshold",
        default_value="0.5",
        description="Quality score threshold for pass/fail",
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
            {"min_area": 0},
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
            {"executed_topic": "/repair/executed"},
            {"executed_path_topic": "/repair/executed_path"},
            {"execute_rate_hz": 10.0},
            {"workspace_width": LaunchConfiguration("workspace_width")},
            {"workspace_height": LaunchConfiguration("workspace_height")},
            {"path_frame": "repairman_map"},
            {"accept_updates_while_executing": False},
            {"path_publish_stride": 5},
            {"publish_progress_path": False},
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
            {"rescan_delay_sec": 2.0},
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="repairman_rviz",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            image_path_arg,
            weights_path_arg,
            yolo_conf_arg,
            workspace_width_arg,
            workspace_height_arg,
            quality_threshold_arg,
            auto_start_arg,
            use_rviz_arg,
            rviz_config_arg,
            connect_world_to_repairman_arg,
            robot_world_frame_arg,
            publish_demo_robot_arg,
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
            rviz_node,
        ]
    )
