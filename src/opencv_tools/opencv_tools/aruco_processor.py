import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from .object_transformation import ObjectTransformation

class ArucoProcessor(Node):
    def __init__(self):
        super().__init__("aruco_processor")
        
        # Declare Parameters
        self.declare_parameter("marker_size", 0.07)
        self.declare_parameter("marker_sizes", "{}")  # JSON mapping, e.g., '{"0": 0.05, "1": 0.1}'
        self.declare_parameter("dictionary", "DICT_ARUCO_ORIGINAL") # Comma-separated list supported
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("stable_tf_id", "aruco_tag")
        self.declare_parameter("target_width", 640)
        self.declare_parameter("target_height", 480)
        
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        sizes_str = self.get_parameter("marker_sizes").get_parameter_value().string_value
        dict_names_str = self.get_parameter("dictionary").get_parameter_value().string_value
        self.stable_tf_id = self.get_parameter("stable_tf_id").get_parameter_value().string_value
        
        # Parse marker sizes
        self.marker_sizes = self.parse_marker_sizes(sizes_str)
        
        # Parse and load ArUco dictionaries
        dict_names = [d.strip() for d in dict_names_str.split(",") if d.strip()]
        self.dictionaries = [self.get_aruco_dictionary(name) for name in dict_names]
        
        # Compatibility check for OpenCV version
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()
            
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.target_width = self.get_parameter("target_width").value
        self.target_height = self.get_parameter("target_height").value
        
        # Diagnostics
        self.img_count = 0
        self.info_received = False
        self.create_timer(2.0, self.diagnostic_timer)
        
        # Subscriptions
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        
        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # Publisher for annotated image, poses, and markers
        self.image_pub = self.create_publisher(Image, "/aruco/image_annotated", 10)
        self.pose_pub = self.create_publisher(PoseArray, "/aruco/poses", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/aruco/markers", 10)
        
        # Initialize Object Transformation
        self.obj_transform = ObjectTransformation()
        
        self.get_logger().info(f"ArUco Processor started. Topics: {image_topic}, {camera_info_topic}")

    def diagnostic_timer(self):
        if not self.info_received:
            self.get_logger().warn("Waiting for CameraInfo...")
        if self.img_count == 0:
            self.get_logger().warn("Waiting for Images...")
        else:
            self.get_logger().info(f"Processing Images... ({self.img_count} in last 2s)")
            self.img_count = 0

    def parse_marker_sizes(self, sizes_str):
        import json
        try:
            sizes = json.loads(sizes_str)
            return {int(k): float(v) for k, v in sizes.items()}
        except Exception as e:
            self.get_logger().error(f"Failed to parse marker_sizes: {e}")
            return {}

    def get_aruco_dictionary(self, name):
        mapping = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }
        if name not in mapping:
            self.get_logger().warn(f"Dictionary '{name}' not found, defaulting to DICT_ARUCO_ORIGINAL")
        
        dict_id = mapping.get(name, cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.get_logger().info(f"Loaded ArUco dictionary: {name if name in mapping else 'DICT_ARUCO_ORIGINAL'}")
        return cv2.aruco.getPredefinedDictionary(dict_id)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        if not self.info_received:
            self.get_logger().info("✅ CameraInfo received!")
            self.info_received = True

    def image_callback(self, msg: Image):
        self.img_count += 1
        if self.camera_matrix is None:
            return
            
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Resize if necessary
        h, w = frame.shape[:2]
        curr_camera_matrix = self.camera_matrix.copy()
        
        if w != self.target_width or h != self.target_height:
            scale_x = self.target_width / w
            scale_y = self.target_height / h
            
            curr_camera_matrix[0, 0] *= scale_x
            curr_camera_matrix[0, 2] *= scale_x
            curr_camera_matrix[1, 1] *= scale_y
            curr_camera_matrix[1, 2] *= scale_y
            
            frame = cv2.resize(frame, (self.target_width, self.target_height))
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detection - try all configured dictionaries
        all_corners = []
        all_ids = []
        
        for dictionary in self.dictionaries:
            if hasattr(cv2.aruco, 'ArucoDetector'):
                detector = cv2.aruco.ArucoDetector(dictionary, self.parameters)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=self.parameters)
            
            if ids is not None:
                all_corners.extend(corners)
                all_ids.extend(ids)
            
        if len(all_ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, all_corners, np.array(all_ids))
            
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            marker_array = MarkerArray()
            
            for i in range(len(all_ids)):
                marker_id = int(all_ids[i][0])
                size = self.marker_sizes.get(marker_id, self.marker_size)
                
                # Pose Estimation Setup
                s = size / 2.0
                # Object points in marker coordinate system: TL, TR, BR, BL
                # This matches OpenCV's coordinate system (X-right, Y-down, Z-out)
                obj_points = np.array([
                    [-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]
                ], dtype=np.float32)

                success, r_vec, t_vec = cv2.solvePnP(
                    obj_points, all_corners[i], curr_camera_matrix, self.dist_coeffs
                )
                
                if success:
                    cv2.drawFrameAxes(frame, curr_camera_matrix, self.dist_coeffs, r_vec, t_vec, size * 0.5)
                    self.draw_3d_bbox(frame, r_vec, t_vec, size, curr_camera_matrix)
                    
                    dist = float(t_vec[2])
                    self.get_logger().info(f"Marker [{marker_id}]: Dist={dist:.2f}m (Size={size:.3f}m)")
                    
                    # Store for PoseArray and Markers
                    rot_mat, _ = cv2.Rodrigues(r_vec)
                    quat = R.from_matrix(rot_mat).as_quat()
                    
                    p = Pose()
                    p.position.x = float(t_vec[0])
                    p.position.y = float(t_vec[1])
                    p.position.z = dist
                    p.orientation.x = float(quat[0])
                    p.orientation.y = float(quat[1])
                    p.orientation.z = float(quat[2])
                    p.orientation.w = float(quat[3])
                    pose_array.poses.append(p)
                    
                    # Create Marker for RViz (3D Box)
                    marker = Marker()
                    marker.header = msg.header
                    marker.ns = "aruco_markers"
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose = p
                    marker.scale.x = size
                    marker.scale.y = size
                    marker.scale.z = 0.005 # Thin for the tag itself
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8
                    marker_array.markers.append(marker)
                    
                    # Publish specific TF
                    self.publish_tf(marker_id, r_vec, t_vec, msg.header.stamp, msg.header.frame_id)
                    
                    # Publish STABLE TF for the first marker found
                    if i == 0 and self.stable_tf_id:
                        self.publish_tf(self.stable_tf_id, r_vec, t_vec, msg.header.stamp, msg.header.frame_id, use_literal=True)

                    # --- OBJECT TRANSFORMATION ---
                    rvec_obj, tvec_obj = self.obj_transform.get_object_pose(r_vec, t_vec)
                    
                    # Log object pose for debugging
                    self.get_logger().info(f"Object from Marker [{marker_id}]: T={tvec_obj.flatten()}")
                    
                    # Draw Object Axes
                    cv2.drawFrameAxes(frame, curr_camera_matrix, self.dist_coeffs, rvec_obj, tvec_obj, 0.05)
                    
                    # Draw Object 3D Box (using Magenta to be very distinct)
                    self.draw_3d_bbox(frame, rvec_obj, tvec_obj, 0.06, curr_camera_matrix, color=(255, 0, 255))
                    
                    # Publish Object TF
                    self.publish_tf(f"object_from_{marker_id}", rvec_obj, tvec_obj, msg.header.stamp, msg.header.frame_id, use_literal=True)
            
            self.pose_pub.publish(pose_array)
            self.marker_pub.publish(marker_array)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def draw_3d_bbox(self, frame, rvec, tvec, size, camera_matrix, color=(0, 255, 0)):
        s = size / 2.0
        # Box corners in marker coord system (Z points out)
        pts_3d = np.array([
            [-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0],
            [-s, -s, size], [s, -s, size], 
            [s, s, size], [-s, s, size]
        ], dtype=np.float32)

        pts_2d, _ = cv2.projectPoints(pts_3d, rvec, tvec, camera_matrix, self.dist_coeffs)
        pts_2d = np.int32(pts_2d).reshape(-1, 2)

        # Base, Top, Pillars
        cv2.polylines(frame, [pts_2d[:4]], True, color, 2)
        cv2.polylines(frame, [pts_2d[4:]], True, color, 2)
        for i in range(4):
            cv2.line(frame, tuple(pts_2d[i]), tuple(pts_2d[i+4]), color, 2)

    def publish_tf(self, marker_id, rvec, tvec, stamp, parent_frame, use_literal=False):
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rot_mat).as_quat()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        
        if use_literal:
            t.child_frame_id = str(marker_id)
        else:
            t.child_frame_id = f"aruco_marker_{marker_id}"
        
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
