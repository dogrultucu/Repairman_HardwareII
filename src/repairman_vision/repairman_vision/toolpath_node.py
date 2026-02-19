import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

from geometry_msgs.msg import Point32, Polygon, PoseStamped
from nav_msgs.msg import Path


class ToolpathNode(Node):
    NEIGHBOR_OFFSETS = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),           (1, 0),
        (-1, 1),  (0, 1),  (1, 1),
    )

    def __init__(self):
        super().__init__("toolpath_node")

        self.declare_parameter("mask_topic", "/damage/mask")
        self.declare_parameter("toolpath_topic", "/repair/toolpath")
        self.declare_parameter("toolpath_path_topic", "/repair/toolpath_path")
        self.declare_parameter("toolpath_overlay_topic", "/repair/toolpath_overlay")
        self.declare_parameter("frame_id", "repairman_map")
        self.declare_parameter("workspace_width", 1.0)
        self.declare_parameter("workspace_height", 1.0)
        self.declare_parameter("output_in_workspace", True)
        self.declare_parameter("workpiece_origin_x", 0.62)
        self.declare_parameter("workpiece_origin_y", 0.25)
        self.declare_parameter("workpiece_width", 0.32)
        self.declare_parameter("workpiece_height", 0.50)
        self.declare_parameter("use_corner_calibration", False)
        self.declare_parameter("corner_a_x", 0.62)
        self.declare_parameter("corner_a_y", 0.25)
        self.declare_parameter("corner_a_z", 0.0)
        self.declare_parameter("corner_b_x", 0.94)
        self.declare_parameter("corner_b_y", 0.25)
        self.declare_parameter("corner_b_z", 0.0)
        self.declare_parameter("corner_c_x", 0.62)
        self.declare_parameter("corner_c_y", 0.75)
        self.declare_parameter("corner_c_z", 0.0)
        self.declare_parameter("path_smooth_window", 9)
        self.declare_parameter("project_to_safe_annulus", True)
        self.declare_parameter("safe_base_x", 0.5)
        self.declare_parameter("safe_base_y", 0.5)
        self.declare_parameter("safe_min_radius", 0.26)
        self.declare_parameter("safe_max_radius", 0.44)
        self.declare_parameter("distance_scale_from_base", 1.0)
        self.declare_parameter("reverse_for_easy_start", True)
        self.declare_parameter("preferred_start_x", 0.75)
        self.declare_parameter("preferred_start_y", 0.50)
        self.declare_parameter("min_area", 300)  # px^2 threshold to ignore tiny blobs
        self.declare_parameter("max_centerline_points", 200)
        self.declare_parameter("mask_open_kernel", 3)
        self.declare_parameter("mask_close_kernel", 3)
        self.declare_parameter("mask_open_iterations", 1)
        self.declare_parameter("mask_close_iterations", 1)
        self.declare_parameter("component_select_mode", "longest_skeleton")
        self.declare_parameter("component_min_area", 80)

        mask_topic = self.get_parameter("mask_topic").value
        toolpath_topic = self.get_parameter("toolpath_topic").value
        self.toolpath_path_topic = self.get_parameter("toolpath_path_topic").value
        self.toolpath_overlay_topic = self.get_parameter("toolpath_overlay_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.workspace_width = float(self.get_parameter("workspace_width").value)
        self.workspace_height = float(self.get_parameter("workspace_height").value)
        self.output_in_workspace = bool(self.get_parameter("output_in_workspace").value)
        self.workpiece_origin_x = float(self.get_parameter("workpiece_origin_x").value)
        self.workpiece_origin_y = float(self.get_parameter("workpiece_origin_y").value)
        self.workpiece_width = float(self.get_parameter("workpiece_width").value)
        self.workpiece_height = float(self.get_parameter("workpiece_height").value)
        self.use_corner_calibration = bool(self.get_parameter("use_corner_calibration").value)
        self.corner_a = (
            float(self.get_parameter("corner_a_x").value),
            float(self.get_parameter("corner_a_y").value),
            float(self.get_parameter("corner_a_z").value),
        )
        self.corner_b = (
            float(self.get_parameter("corner_b_x").value),
            float(self.get_parameter("corner_b_y").value),
            float(self.get_parameter("corner_b_z").value),
        )
        self.corner_c = (
            float(self.get_parameter("corner_c_x").value),
            float(self.get_parameter("corner_c_y").value),
            float(self.get_parameter("corner_c_z").value),
        )
        self.corner_ab = (
            self.corner_b[0] - self.corner_a[0],
            self.corner_b[1] - self.corner_a[1],
            self.corner_b[2] - self.corner_a[2],
        )
        self.corner_ac = (
            self.corner_c[0] - self.corner_a[0],
            self.corner_c[1] - self.corner_a[1],
            self.corner_c[2] - self.corner_a[2],
        )
        ab_len = float(np.linalg.norm(np.array(self.corner_ab[:2], dtype=float)))
        ac_len = float(np.linalg.norm(np.array(self.corner_ac[:2], dtype=float)))
        if self.use_corner_calibration and (ab_len < 1e-6 or ac_len < 1e-6):
            self.get_logger().warn(
                "Corner calibration is enabled but corner geometry is degenerate. "
                "Falling back to origin/width/height mapping."
            )
            self.use_corner_calibration = False
        self.path_smooth_window = int(self.get_parameter("path_smooth_window").value)
        self.project_to_safe_annulus = bool(
            self.get_parameter("project_to_safe_annulus").value
        )
        self.safe_base_x = float(self.get_parameter("safe_base_x").value)
        self.safe_base_y = float(self.get_parameter("safe_base_y").value)
        self.safe_min_radius = float(self.get_parameter("safe_min_radius").value)
        self.safe_max_radius = float(self.get_parameter("safe_max_radius").value)
        self.distance_scale_from_base = max(
            0.0, float(self.get_parameter("distance_scale_from_base").value)
        )
        self.reverse_for_easy_start = bool(self.get_parameter("reverse_for_easy_start").value)
        self.preferred_start_x = float(self.get_parameter("preferred_start_x").value)
        self.preferred_start_y = float(self.get_parameter("preferred_start_y").value)
        self.min_area = int(self.get_parameter("min_area").value)
        self.max_centerline_points = int(self.get_parameter("max_centerline_points").value)
        self.mask_open_kernel = self.to_odd_int(
            self.get_parameter("mask_open_kernel").value, minimum=1
        )
        self.mask_close_kernel = self.to_odd_int(
            self.get_parameter("mask_close_kernel").value, minimum=1
        )
        self.mask_open_iterations = max(
            0, int(self.get_parameter("mask_open_iterations").value)
        )
        self.mask_close_iterations = max(
            0, int(self.get_parameter("mask_close_iterations").value)
        )
        self.component_select_mode = str(
            self.get_parameter("component_select_mode").value
        ).strip().lower()
        self.component_min_area = max(
            1, int(self.get_parameter("component_min_area").value)
        )
        if self.use_corner_calibration and self.project_to_safe_annulus:
            self.get_logger().warn(
                "Disabling project_to_safe_annulus while use_corner_calibration=true "
                "to preserve real A/B/C geometry."
            )
            self.project_to_safe_annulus = False
        self.has_ximgproc = hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, mask_topic, self.cb, 10)
        self.pub = self.create_publisher(Polygon, toolpath_topic, 10)
        self.pub_path = self.create_publisher(Path, self.toolpath_path_topic, 10)
        self.pub_overlay = self.create_publisher(Image, self.toolpath_overlay_topic, 10)

        self.get_logger().info(
            f"ToolpathNode subscribed: {mask_topic} -> publishes: {toolpath_topic}, "
            f"{self.toolpath_path_topic}, {self.toolpath_overlay_topic}"
        )
        self.get_logger().info(
            f"Centerline mode: skeleton-based ({'ximgproc' if self.has_ximgproc else 'morph-fallback'})"
        )
        self.get_logger().info(
            f"Component select mode: {self.component_select_mode}, min area: {self.component_min_area}px"
        )
        if self.use_corner_calibration:
            self.get_logger().info(
                "Using 3-corner calibration (A/B/C) for pixel -> workspace mapping."
            )

    def cb(self, msg: Image):
        """Generate a true centerline toolpath from the detected crack mask."""
        raw_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        mask = raw_mask.copy()
        if self.mask_open_kernel > 1 and self.mask_open_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (self.mask_open_kernel, self.mask_open_kernel)
                ),
                iterations=self.mask_open_iterations,
            )
        if self.mask_close_kernel > 1 and self.mask_close_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (self.mask_close_kernel, self.mask_close_kernel)
                ),
                iterations=self.mask_close_iterations,
            )
        binary = np.where(mask > 0, 255, 0).astype(np.uint8)

        component_mask, area, precomputed_skeleton = self.extract_best_component(binary)
        if component_mask is None or area < self.min_area:
            return

        skeleton = (
            precomputed_skeleton
            if precomputed_skeleton is not None
            else self.skeletonize(component_mask)
        )
        ordered_centerline = self.extract_ordered_centerline(skeleton)
        if len(ordered_centerline) < 2:
            self.get_logger().debug("Centerline extraction produced < 2 points, skipping publish.")
            return

        sampled = self.downsample_path(ordered_centerline)
        world_points = self.to_workspace_points(
            sampled, raw_mask.shape[1], raw_mask.shape[0]
        )
        world_points = self.smooth_world_path(world_points)
        world_points = self.scale_path_from_base(world_points)
        if self.project_to_safe_annulus:
            world_points = self.project_path_to_safe_annulus(world_points)
        if self.reverse_for_easy_start:
            world_points = self.reverse_if_better_start(world_points)
        if len(world_points) < 2:
            return

        poly = Polygon()
        for x, y, z in world_points:
            poly.points.append(Point32(x=float(x), y=float(y), z=float(z)))

        self.pub.publish(poly)
        self.publish_path_preview(world_points, msg.header)
        self.publish_overlay(component_mask, skeleton, sampled, msg.header, area)

    def extract_best_component(self, binary):
        """Select one component for centerline extraction based on configured mode."""
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        if num_labels <= 1:
            return None, 0, None

        label_ids = [
            label for label in range(1, num_labels)
            if int(stats[label, cv2.CC_STAT_AREA]) >= self.component_min_area
        ]
        if not label_ids:
            label_ids = list(range(1, num_labels))

        best_component = None
        best_area = 0
        best_score = float("-inf")
        best_skeleton = None

        for label in label_ids:
            area = int(stats[label, cv2.CC_STAT_AREA])
            component = np.where(labels == label, 255, 0).astype(np.uint8)

            if self.component_select_mode == "largest_area":
                score = float(area)
                skeleton = None
            else:
                skeleton = self.skeletonize(component)
                score = float(cv2.countNonZero(skeleton))

            if score > best_score:
                best_score = score
                best_component = component
                best_area = area
                best_skeleton = skeleton

        return best_component, best_area, best_skeleton

    def skeletonize(self, binary):
        """Skeletonize crack mask so centerline passes through the crack middle."""
        if self.has_ximgproc:
            return cv2.ximgproc.thinning(binary)

        # Morphological fallback when ximgproc is unavailable.
        skel = np.zeros_like(binary)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        work = binary.copy()
        for _ in range(2048):
            eroded = cv2.erode(work, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(work, temp)
            skel = cv2.bitwise_or(skel, temp)
            work = eroded
            if cv2.countNonZero(work) == 0:
                break
        return skel

    def neighbors(self, point, point_set):
        """Yield 8-connected neighbors for a skeleton pixel."""
        x, y = point
        for dx, dy in self.NEIGHBOR_OFFSETS:
            npnt = (x + dx, y + dy)
            if npnt in point_set:
                yield npnt

    def bfs_tree(self, point_set, start):
        """Breadth-first traversal on skeleton graph."""
        q = deque([start])
        parent = {start: None}
        dist = {start: 0}
        farthest = start

        while q:
            node = q.popleft()
            if dist[node] > dist[farthest]:
                farthest = node
            for nb in self.neighbors(node, point_set):
                if nb in dist:
                    continue
                dist[nb] = dist[node] + 1
                parent[nb] = node
                q.append(nb)

        return parent, dist, farthest

    def reconstruct_path(self, parent, start, goal):
        """Reconstruct path from BFS parent map."""
        if goal not in parent:
            return []
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        path.reverse()
        if not path or path[0] != start:
            return []
        return path

    def extract_ordered_centerline(self, skeleton):
        """Convert skeleton pixels into an ordered centerline polyline."""
        ys, xs = np.where(skeleton > 0)
        if len(xs) < 2:
            return []
        point_set = set((int(x), int(y)) for x, y in zip(xs, ys))

        degrees = {}
        endpoints = []
        for p in point_set:
            deg = sum(1 for _ in self.neighbors(p, point_set))
            degrees[p] = deg
            if deg == 1:
                endpoints.append(p)

        # Preferred case: open crack with two (or more) endpoints.
        if len(endpoints) >= 2:
            best_start = endpoints[0]
            best_end = endpoints[1]
            best_len = -1
            for s in endpoints:
                _, dist, _ = self.bfs_tree(point_set, s)
                for e in endpoints:
                    if e == s or e not in dist:
                        continue
                    if dist[e] > best_len:
                        best_len = dist[e]
                        best_start = s
                        best_end = e
            parent, _, _ = self.bfs_tree(point_set, best_start)
            path = self.reconstruct_path(parent, best_start, best_end)
            return path

        # Fallback for loops/branchy skeletons: use graph diameter endpoints.
        start = next(iter(point_set))
        _, _, a = self.bfs_tree(point_set, start)
        parent, _, b = self.bfs_tree(point_set, a)
        path = self.reconstruct_path(parent, a, b)
        return path

    def downsample_path(self, points):
        """Limit point count while preserving full path span."""
        if len(points) <= self.max_centerline_points:
            return points
        idx = np.linspace(0, len(points) - 1, self.max_centerline_points, dtype=int)
        idx = np.unique(idx)
        return [points[i] for i in idx]

    def publish_overlay(self, component_mask, skeleton, sampled, header, area):
        """Publish a debug image that overlays the generated centerline path."""
        overlay = cv2.cvtColor(component_mask, cv2.COLOR_GRAY2BGR)
        skel_y, skel_x = np.where(skeleton > 0)
        overlay[skel_y, skel_x] = (0, 0, 255)

        if len(sampled) >= 2:
            pts = np.array(sampled, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(overlay, [pts], isClosed=False, color=(0, 255, 255), thickness=2)
            cv2.circle(overlay, (int(sampled[0][0]), int(sampled[0][1])), 3, (255, 0, 0), -1)
            cv2.circle(overlay, (int(sampled[-1][0]), int(sampled[-1][1])), 3, (0, 255, 0), -1)

        cv2.putText(
            overlay,
            f"area={int(area)} centerline_pts={len(sampled)}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        out = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        out.header = header
        self.pub_overlay.publish(out)

    def to_workspace_points(self, sampled, image_width, image_height):
        """Map image-centerline pixels into workspace/world coordinates."""
        if not sampled:
            return []

        if not self.output_in_workspace:
            return [(float(x), float(y), 0.0) for x, y in sampled]

        denom_x = max(1.0, float(image_width - 1))
        denom_y = max(1.0, float(image_height - 1))
        world = []
        for x, y in sampled:
            u = float(x) / denom_x
            v = float(y) / denom_y
            if self.use_corner_calibration:
                wx = self.corner_a[0] + u * self.corner_ab[0] + v * self.corner_ac[0]
                wy = self.corner_a[1] + u * self.corner_ab[1] + v * self.corner_ac[1]
                wz = self.corner_a[2] + u * self.corner_ab[2] + v * self.corner_ac[2]
            else:
                wx = self.workpiece_origin_x + u * self.workpiece_width
                wy = self.workpiece_origin_y + v * self.workpiece_height
                wz = 0.0
                wx = float(np.clip(wx, 0.0, self.workspace_width))
                wy = float(np.clip(wy, 0.0, self.workspace_height))
            world.append((float(wx), float(wy), float(wz)))
        return world

    def smooth_world_path(self, points):
        if len(points) < 3:
            return points
        window = max(1, int(self.path_smooth_window))
        if window < 3:
            return points
        if window % 2 == 0:
            window += 1
        if len(points) <= window:
            return points

        arr = np.array(points, dtype=np.float32)
        half = window // 2
        padded = np.pad(arr, ((half, half), (0, 0)), mode="edge")
        kernel = np.ones(window, dtype=np.float32) / float(window)
        sx = np.convolve(padded[:, 0], kernel, mode="valid")
        sy = np.convolve(padded[:, 1], kernel, mode="valid")
        sz = np.convolve(padded[:, 2], kernel, mode="valid")
        smoothed = np.stack([sx, sy, sz], axis=1)
        smoothed[0] = arr[0]
        smoothed[-1] = arr[-1]
        return [(float(p[0]), float(p[1]), float(p[2])) for p in smoothed]

    def project_path_to_safe_annulus(self, points):
        if not points:
            return points
        r_min = min(self.safe_min_radius, self.safe_max_radius)
        r_max = max(self.safe_min_radius, self.safe_max_radius)
        out = []
        for x, y, z in points:
            dx = float(x) - self.safe_base_x
            dy = float(y) - self.safe_base_y
            r = float(np.hypot(dx, dy))
            if r < 1e-6:
                dx = 1.0
                dy = 0.0
                r = 1.0
            r_safe = float(np.clip(r, r_min, r_max))
            scale = r_safe / r
            nx = self.safe_base_x + dx * scale
            ny = self.safe_base_y + dy * scale
            nx = float(np.clip(nx, 0.0, self.workspace_width))
            ny = float(np.clip(ny, 0.0, self.workspace_height))
            out.append((nx, ny, float(z)))
        return out

    def reverse_if_better_start(self, points):
        if len(points) < 2:
            return points
        px, py = self.preferred_start_x, self.preferred_start_y
        d0 = (points[0][0] - px) ** 2 + (points[0][1] - py) ** 2
        d1 = (points[-1][0] - px) ** 2 + (points[-1][1] - py) ** 2
        if d1 + 1e-9 < d0:
            return list(reversed(points))
        return points

    def scale_path_from_base(self, points):
        """Scale toolpath radial distance from base center."""
        if not points:
            return points
        scale = float(self.distance_scale_from_base)
        if abs(scale - 1.0) < 1e-6:
            return points
        out = []
        for x, y, z in points:
            nx = self.safe_base_x + (float(x) - self.safe_base_x) * scale
            ny = self.safe_base_y + (float(y) - self.safe_base_y) * scale
            out.append((float(nx), float(ny), float(z)))
        return out

    def publish_path_preview(self, world_points, header):
        """Publish RViz-friendly preview of the generated toolpath as nav_msgs/Path."""
        if not world_points:
            return

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = header.stamp

        for i, (x, y, z) in enumerate(world_points):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)

            if i < len(world_points) - 1:
                nx, ny, _ = world_points[i + 1]
                dx = nx - x
                dy = ny - y
                yaw = np.arctan2(dy, dx)
                pose.pose.orientation.z = float(np.sin(yaw / 2.0))
                pose.pose.orientation.w = float(np.cos(yaw / 2.0))
            else:
                pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub_path.publish(path)

    @staticmethod
    def to_odd_int(value, minimum=1):
        out = max(minimum, int(value))
        if out % 2 == 0:
            out += 1
        return out


def main():
    rclpy.init()
    node = ToolpathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
