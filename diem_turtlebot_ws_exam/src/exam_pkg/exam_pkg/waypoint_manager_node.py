import rclpy
from rclpy.node import Node
from my_msg_pkg.msg import ConeArray
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from collections import defaultdict, deque
import time
import threading
import tf2_ros
import tf2_geometry_msgs
import math

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')

        self.get_logger().info(f"Waypoint manager initialized")

        self.cones_sub = self.create_subscription(
            ConeArray, '/detected_cones', self.cones_callback, 10)
        self.goal_sub = self.create_subscription(
            Bool, '/intermediate_goal_reached', self.goal_callback, 10)

        self.waypoint_pub = self.create_publisher(PoseStamped, '/intermediate_goals', 10)

        self.can_publish = True
        self.first_goal_published = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.buffer_lock = threading.Lock()
        self.waypoint_buffer = deque()
        self.published_waypoints = {}
        self.position_tolerance = 0.3
        self.averaging_window_seconds = 2.5
        self.min_detections = 2

        self.create_timer(0.2, self.check_and_publish)

    def cones_callback(self, msg):
        if not self.can_publish and self.first_goal_published:
            return

        yellow_cones = []
        orange_red_cones = []

        for cone in msg.cones:
            if cone.color in ["yellow"]:
                yellow_cones.append(cone.pose)
            elif cone.color in ["orange", "red"]:
                orange_red_cones.append(cone.pose)

        waypoint_pose_non_stamped = self.calculate_waypoint_on_camera(yellow_cones, orange_red_cones)

        if waypoint_pose_non_stamped is None:
            return

        transformed = self.transform_pose_to_map(waypoint_pose_non_stamped)

        if transformed:
            wp_id = self.position_to_id(transformed.pose.position)
            angle_deg = math.degrees(math.atan2(transformed.pose.position.y, transformed.pose.position.x))
            self.get_logger().info(f"Waypoint map position - id {wp_id}: X={transformed.pose.position.x:.2f}, Y={transformed.pose.position.y:.2f}, Angle={angle_deg:.2f}")
            self.add_to_buffer({
                'pose': transformed,
                'id': wp_id,
                'angle': angle_deg
            })

    def transform_pose_to_map(self, pose_non_stamped):
        try:
            if not self.tf_buffer.can_transform("map", "oakd_rgb_camera_optical_frame", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn("Transform oakd_rgb_camera_optical_frame -> map not available yet")
                return None

            transform = self.tf_buffer.lookup_transform(
                "map", "oakd_rgb_camera_optical_frame", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_non_stamped, transform)

            new_pose_stamped = PoseStamped()
            new_pose_stamped.header.frame_id = "map"
            new_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            new_pose_stamped.pose = transformed_pose

            return new_pose_stamped

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF transform failed")
            return None

    def calculate_waypoint_on_camera(self, yellow_cones, orange_red_cones):
        offset_meters = 0.5
        yellow_cones_forward = [c for c in yellow_cones if c.position.z > 0]
        orange_red_cones_forward = [c for c in orange_red_cones if c.position.z > 0]

        waypoint_pose_non_stamped = Pose()

        if yellow_cones_forward and orange_red_cones_forward:
            closest_yellow = min(yellow_cones_forward, key=lambda c: c.position.z)
            closest_orange_red = min(orange_red_cones_forward, key=lambda c: c.position.z)

            z_diff = abs(closest_yellow.position.z - closest_orange_red.position.z)

            if z_diff < 1.0:
                waypoint_pose_non_stamped.position.x = (closest_yellow.position.x + closest_orange_red.position.x) / 2
                waypoint_pose_non_stamped.position.y = (closest_yellow.position.y + closest_orange_red.position.y) / 2
                waypoint_pose_non_stamped.position.z = (closest_yellow.position.z + closest_orange_red.position.z) / 2
            else:
                chosen = closest_yellow if closest_yellow.position.z < closest_orange_red.position.z else closest_orange_red
                waypoint_pose_non_stamped.position.x = chosen.position.x + (offset_meters if chosen == closest_yellow else -offset_meters)
                waypoint_pose_non_stamped.position.y = chosen.position.y
                waypoint_pose_non_stamped.position.z = chosen.position.z

        elif yellow_cones_forward:
            closest_yellow = min(yellow_cones_forward, key=lambda c: c.position.z)
            waypoint_pose_non_stamped.position.x = closest_yellow.position.x + offset_meters
            waypoint_pose_non_stamped.position.y = closest_yellow.position.y
            waypoint_pose_non_stamped.position.z = closest_yellow.position.z

        elif orange_red_cones_forward:
            closest_orange_red = min(orange_red_cones_forward, key=lambda c: c.position.z)
            waypoint_pose_non_stamped.position.x = closest_orange_red.position.x - offset_meters
            waypoint_pose_non_stamped.position.y = closest_orange_red.position.y
            waypoint_pose_non_stamped.position.z = closest_orange_red.position.z
        else:
            return None

        waypoint_pose_non_stamped.orientation.w = 1.0
        self.get_logger().info(f"Waypoint camera position: X={waypoint_pose_non_stamped.position.x:.2f}, Y={waypoint_pose_non_stamped.position.y:.2f}, Z={waypoint_pose_non_stamped.position.z:.2f} meters")
        return waypoint_pose_non_stamped

    def position_to_id(self, position):
        x_id = round(position.x / self.position_tolerance)
        y_id = round(position.y / self.position_tolerance)
        return f"{x_id}_{y_id}"

    def add_to_buffer(self, waypoint):
        with self.buffer_lock:
            self.waypoint_buffer.append({
                'id': waypoint['id'],
                'pose': waypoint['pose'],
                'angle': waypoint['angle'],
                'timestamp': time.time()
            })
            cutoff = time.time() - self.averaging_window_seconds
            while self.waypoint_buffer and self.waypoint_buffer[0]['timestamp'] < cutoff:
                self.waypoint_buffer.popleft()

    def is_waypoint_published(self, wp_id, position, angle, pos_thresh=1.0, angle_thresh=5.0):
        if wp_id not in self.published_waypoints:
            return False
        old = self.published_waypoints[wp_id]
        dx = old['x'] - position[0]
        dy = old['y'] - position[1]
        dist = math.hypot(dx, dy)
        angle_diff = abs(old['angle'] - angle)
        return dist < pos_thresh and angle_diff < angle_thresh

    def check_and_publish(self):
        with self.buffer_lock:
            if len(self.waypoint_buffer) < self.min_detections:
                return

            groups = defaultdict(list)
            for wp in self.waypoint_buffer:
                groups[wp['id']].append(wp)

            best_id, best_count = None, 0
            for wp_id, detections in groups.items():
                if len(detections) > best_count:
                    best_count = len(detections)
                    best_id = wp_id

            if best_id is not None and best_count > 0:
                detections = groups[best_id]
                avg_x = sum(d['pose'].pose.position.x for d in detections) / len(detections)
                avg_y = sum(d['pose'].pose.position.y for d in detections) / len(detections)
                avg_angle = sum(d['angle'] for d in detections) / len(detections)

                if self.is_waypoint_published(best_id, (avg_x, avg_y), avg_angle):
                    self.get_logger().info(f"Skipping duplicate waypoint near previous {best_id}")
                    return

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = avg_x
                pose.pose.position.y = avg_y
                pose.pose.orientation.w = 1.0

                self.get_logger().info(f"Published intermediate waypoint map position - id: {best_id}: X={avg_x:.2f}, Y={avg_y:.2f}, Angle={avg_angle:.2f}")

                self.published_waypoints[best_id] = {
                    'x': avg_x,
                    'y': avg_y,
                    'angle': avg_angle,
                    'timestamp': time.time()
                }

                self.waypoint_pub.publish(pose)

                self.can_publish = False
                self.first_goal_published = True
                self.waypoint_buffer.clear()

    def goal_callback(self, msg):
        self.get_logger().info("Intermediate goal reached. Ready for next waypoint.")
        self.can_publish = True

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
