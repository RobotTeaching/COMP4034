import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
import cv2
import pyrealsense2 as rs

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_SIZE_MM = 250  # size of marker in mm

def quaternion_to_rpy(q: Quaternion):
    """
    Convert a quaternion into roll, pitch, and yaw (in radians).
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class ImageSubscriber(Node):
    
    def __init__(self):
        super().__init__('image_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw/compressed',
            self.image_callback,
            10)
        
        self.camera_matrix = None
        self.distortion_coefficients = None
        
        self.image = []

        
        self.br = CvBridge() # We need this to convert ros2 Image to cv2 Image
        
        self.timer = self.create_timer(0.2, self.timer_callback)
    
    def image_callback(self, data):
        self.get_logger().info('Receiving video frame')
    
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        self.image = current_frame

    def camera_info_callback(self, data):
        self.get_logger().info('Receiving camera info')
    
        self.camera_matrix = np.array(msg.K).reshape((3,3))
        self.distortion_coefficients = np.array(msg.D)
        
    def timer_callback(self):
        current_frame = self.image
        if current_frame == []:
            return

        grey = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY) # aruco detection works better in grayscale!
        
        corners, marker_ids, _ = cv2.aruco.detectMarkers(current_frame, ARUCO_DICT) # Detect aurco markers
        current_frame = cv2.aruco.drawDetectedMarkers(current_frame, corners, ids)
        
        if marker_ids is not None:
            for i in range(0, len(marker_ids)):
                rvec, tvec, markerpoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE_MM / 1000, self.camera_matrix, self.distortion_coefficents)
                translation = tvec.flatten()
                rotation = rvec.ravel()

                self.get_logger().info(f"Detected Aruco with ID {marker_ids[i]} at position {translation}, and rotation {rotation}")
                
                current_frame = cv2.aruco.drawAxis(current_frame, self.camera_matrix, self.distortion_coefficients, rvec, tvec, MARKER_SIZE_MM / 1000)
                
        
        cv2.imshow("ARUCO DETECTIONS", current_frame)
        cv2.waitKey(1)
    

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
