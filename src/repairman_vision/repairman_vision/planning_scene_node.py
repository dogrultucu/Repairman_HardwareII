import rclpy
from rclpy.node import Node

try:
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    MOVEIT_MSGS_AVAILABLE = True
except ImportError:
    CollisionObject = None
    PlanningScene = None
    SolidPrimitive = None
    Pose = None
    MOVEIT_MSGS_AVAILABLE = False


class PlanningSceneNode(Node):
    """Publish static collision objects into the MoveIt planning scene."""

    def __init__(self):
        super().__init__("planning_scene_node")

        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("frame_id", "repairman_map")
        self.declare_parameter("publish_rate_hz", 0.5)

        self.declare_parameter("table_size", [1.2, 1.2, 0.08])
        self.declare_parameter("table_pose", [0.5, 0.5, -0.06])

        self.declare_parameter("workpiece_size", [0.22, 0.40, 0.04])
        self.declare_parameter("workpiece_pose", [0.5, 0.5, 0.03])

        self.declare_parameter("rear_keepout_size", [0.2, 1.1, 0.6])
        self.declare_parameter("rear_keepout_pose", [0.1, 0.5, 0.30])

        self.topic = str(self.get_parameter("planning_scene_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))

        self.table_size = [float(v) for v in self.get_parameter("table_size").value]
        self.table_pose = [float(v) for v in self.get_parameter("table_pose").value]

        self.workpiece_size = [float(v) for v in self.get_parameter("workpiece_size").value]
        self.workpiece_pose = [float(v) for v in self.get_parameter("workpiece_pose").value]

        self.rear_keepout_size = [float(v) for v in self.get_parameter("rear_keepout_size").value]
        self.rear_keepout_pose = [float(v) for v in self.get_parameter("rear_keepout_pose").value]

        self.enabled = MOVEIT_MSGS_AVAILABLE
        if not self.enabled:
            self.get_logger().error(
                "moveit_msgs not found. Install MoveIt packages and relaunch planning_scene_node."
            )
            return

        self.pub = self.create_publisher(PlanningScene, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_scene)

        self.get_logger().info(
            f"Publishing MoveIt planning scene objects on {self.topic} in frame {self.frame_id}"
        )
        self.publish_scene()

    def _make_box(self, object_id: str, size, pose_xyz):
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self.frame_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        pose = Pose()
        pose.position.x = float(pose_xyz[0])
        pose.position.y = float(pose_xyz[1])
        pose.position.z = float(pose_xyz[2])
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    def publish_scene(self):
        if not self.enabled:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.world.collision_objects.append(
            self._make_box("repair_table", self.table_size, self.table_pose)
        )
        scene.world.collision_objects.append(
            self._make_box("repair_workpiece", self.workpiece_size, self.workpiece_pose)
        )
        scene.world.collision_objects.append(
            self._make_box("rear_keepout", self.rear_keepout_size, self.rear_keepout_pose)
        )

        self.pub.publish(scene)


def main():
    rclpy.init()
    node = PlanningSceneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
