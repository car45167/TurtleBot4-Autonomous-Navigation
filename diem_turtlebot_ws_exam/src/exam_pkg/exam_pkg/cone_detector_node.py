import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from my_msg_pkg.msg import Cone, ConeArray
import os
from ament_index_python.packages import get_package_share_directory

class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector_node')

        self.get_logger().info(f"Cone detector initializing")

        self.bridge = CvBridge()

        self.visualization = True

        self.window_name = "Cone Detection View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.startWindowThread()

        def get_cone_model_path():
            pkg_path = get_package_share_directory('exam_pkg')
            model_path = os.path.join(pkg_path, 'resource', 'Cone.pt')
            return model_path

        cone_model_path = get_cone_model_path()
        self.model = YOLO(cone_model_path)
        self.get_logger().info(f"Model correctly loaded")
        
        # Subscriptions
        self.rgb_sub = self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/oakd/stereo/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/oakd/rgb/preview/camera_info', self.camera_info_callback, 10)
        
        # Publisher for ConeArray
        self.cones_pub = self.create_publisher(ConeArray, '/detected_cones', 10)
        
        # Latest images and camera info
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        
        # Image dimension placeholders
        self.rgb_height = 400
        self.rgb_width = 400
        self.depth_height = 720
        self.depth_width = 720

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Resize RGB to fixed square size (400x400)
            self.latest_rgb = cv2.resize(cv_image, (400, 400))
            self.rgb_height, self.rgb_width = self.latest_rgb.shape[:2]
            
            # Try processing detections if depth and camera info are ready
            if self.latest_depth is not None and self.camera_info is not None:
                self.process_detections()
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def depth_callback(self, msg):
        try:
            if msg.encoding == '16UC1':
                depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
                depth_img /= 1000.0  # Convert mm to meters
            elif msg.encoding == '32FC1':
                depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            else:
                self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
                return
            
            # Center crop depth to square matching RGB size
            start_x = (depth_img.shape[1] - depth_img.shape[0]) // 2
            self.latest_depth = depth_img[:, start_x:start_x + depth_img.shape[0]]
            self.depth_height, self.depth_width = self.latest_depth.shape[:2]
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def process_detections(self):
        if self.latest_rgb is None or self.latest_depth is None or self.camera_info is None:
            self.get_logger().info("Waiting for both RGB and depth images...", throttle_duration_sec=1.0)
            return None
        try:
            results = self.model.predict(source=self.latest_rgb, conf=0.6, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Model prediction failed: {e}")
            return
        
        frame = self.latest_rgb.copy()
        
        detections = ConeArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box.astype(int)
                center_x = (x1 + x2) // 2
                center_y = y2
                
                # Get dominant color in the detected bounding box
                color = self.get_dominant_color(self.latest_rgb[y1:y2, x1:x2])
                
                # Calculate 3D pose from depth image
                pose = self.calculate_cone_pose(center_x, center_y)
                if pose is None:
                    continue
                
                # Create and append cone detection message
                cone = Cone()
                cone.color = color
                cone.pose = pose.pose
                detections.cones.append(cone)

                if self.visualization:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label_color = f"Cone {color}"
                    label_x = f"X {pose.pose.position.x:.2f}"
                    label_y = f"Y {pose.pose.position.y:.2f}"
                    label_z = f"Z {pose.pose.position.z:.2f}"
                    cv2.putText(frame, label_color, (x1, y1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(frame, label_x, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(frame, label_y, (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(frame, label_z, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if self.visualization:
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        
        # Publish all detected cones
        self.cones_pub.publish(detections)

    def calculate_cone_pose(self, x_rgb, y_rgb):
        x_depth, y_depth = self.calculate_depth_coordinates_from_rgb_coordinates(x_rgb, y_rgb)
        pose = self.calculate_cone_pose_from_depth((x_depth, y_depth), self.latest_depth)
        return pose

    def calculate_depth_coordinates_from_rgb_coordinates(self, x_rgb, y_rgb):
        if self.depth_height is None or self.rgb_height is None or self.depth_width is None or self.rgb_width is None:
            self.get_logger().warn("Image dimensions not set, cannot convert coordinates.")
            return 0, 0
        
        scale_x = self.depth_width / self.rgb_width
        scale_y = self.depth_height / self.rgb_height
        
        x_depth = int(x_rgb * scale_x)
        y_depth = int(y_rgb * scale_y)

        x_depth = max(0, min(x_depth, self.depth_width - 1))
        y_depth = max(0, min(y_depth, self.depth_height - 1))
    
        return x_depth, y_depth

    def calculate_cone_pose_from_depth(self, pixel_on_depth, depth_img):
        x_px, y_px = pixel_on_depth
        h, w = depth_img.shape

        if x_px < 0 or x_px >= w or y_px < 0 or y_px >= h:
            self.get_logger().warn(f"Pixel coordinates ({x_px}, {y_px}) out of bounds for depth image {w}x{h}")
            return None

        # Local median filtering to reduce noise in depth
        window_x = 5
        window_y = 2

        x_min = max(0, x_px - window_x)
        x_max = min(w - 1, x_px + window_x)
        y_min = max(0, y_px - window_y)
        y_max = min(h - 1, y_px + window_y)

        patch = depth_img[y_min:y_max+1, x_min:x_max+1]

        valid_patch = patch[(patch > 0.1) & (patch < 10.0)]
        if valid_patch.size == 0:
            return None

        Z = float(np.median(valid_patch))
        if Z <= 0.3 or Z > 6:
            # Depth out of valid range
            return None

        if self.camera_info is None:
            self.get_logger().warn("Camera info not yet received.")
            return None

        K = self.camera_info.k
        fx_orig, fy_orig = K[0], K[4]
        cx_orig, cy_orig = K[2], K[5]
        w_orig = self.camera_info.width
        h_orig = self.camera_info.height

        # Scale intrinsics to depth image size
        fx = fx_orig * (w / w_orig)
        fy = fy_orig * (h / h_orig)
        cx = cx_orig * (w / w_orig)
        cy = cy_orig * (h / h_orig)

        X_cam = (x_px - cx) * Z / fx        # X points right , x growing means checkpoint goes right
        Y_cam = (y_px - cy) * Z / fy        # Y points down , y growing means checkpoint goes down
        Z_cam = Z                           # Z points forward : depth

        self.get_logger().info(f"Cone camera position: X={X_cam:.2f}, Y={Y_cam:.2f}, Z={Z_cam:.2f} meters")

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'oakd_rgb_camera_optical_frame'
        pose_msg.pose.position.x = X_cam
        pose_msg.pose.position.y = Y_cam
        pose_msg.pose.position.z = Z_cam
        pose_msg.pose.orientation.w = 1.0

        return pose_msg

    def get_dominant_color(self, crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        colors = {
            'red': [
                ((0, 80, 50), (5, 255, 255)),
                ((161, 80, 50), (180, 255, 255)) 
            ],
            'orange': [((6, 80, 50), (20, 255, 255))],
            'yellow': [((21, 60, 50), (40, 255, 255))],
            'green': [((41, 70, 50), (85, 255, 255))],
            'blue': [((86, 70, 50), (130, 255, 255))]
        }
        max_color = 'undefined'
        max_pixels = 0
        for color_name, ranges in colors.items():
            mask_total = None
            for lower, upper in ranges:
                lower_np = np.array(lower)
                upper_np = np.array(upper)
                mask = cv2.inRange(hsv, lower_np, upper_np)
                if mask_total is None:
                    mask_total = mask
                else:
                    mask_total = cv2.bitwise_or(mask_total, mask)
            count = cv2.countNonZero(mask_total)
            if count > max_pixels:
                max_pixels = count
                max_color = color_name
        return max_color

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
