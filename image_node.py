import rclpy
import math
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
import cv2
import pyrealsense2 as rs

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_SIZE_MM = 200  # size of marker in mm

class ImageSubscriber(Node):
    
    def __init__(self):
        super().__init__('image_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription_image = self.create_subscription(
            CompressedImage,
            '/camera_depth/image_raw/compressed', # !! This topic is for the SIMULATED robot. You'll need to change it for real life
            self.image_callback,
            qos_profile_sensor_data)
        
        self.subscription_camerainfo = self.create_subscription(
                    CameraInfo,
                    '/camera_depth/camera_info', # !! This topic is for the SIMULATED robot. You'll need to change it for real life
                    self.camera_info_callback,
                    qos_profile_sensor_data)
        
        self.camera_matrix = None
        self.distortion_coefficients = None
        
        self.image = []

        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def image_callback(self, msg: CompressedImage):
        # self.get_logger().info('Receiving Compressed Image Frame')
        
        # Logic to convert Compressed Images to CV2
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
        if frame is None:
            self.get_logger().info('Failed to decode image')
            return
        
        self.image = frame
        
        
    def camera_info_callback(self, msg: CameraInfo):
        #self.get_logger().info('Receiving camera info')
    
        self.camera_matrix = np.array(msg.k).reshape((3,3))
        self.distortion_coefficients = np.array(msg.d)
        
    def timer_callback(self):
        current_frame = self.image
        if current_frame == []:
            return

        grey = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY) # aruco detection works better in grayscale!
        
        corners, marker_ids, _ = cv2.aruco.detectMarkers(current_frame, ARUCO_DICT) # Detect aurco markers
        current_frame = cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
        
        if marker_ids is not None and self.camera_matrix is not None:
            for i in range(0, len(marker_ids)):
                rvec, tvec, markerpoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE_MM / 1000, self.camera_matrix, self.distortion_coefficients)
                translation = tvec.flatten()
                rotation = rvec.ravel()

                self.get_logger().info(f"Detected Aruco with ID {marker_ids[i]} at position {translation}, and rotation {rotation}")
                
                current_frame = cv2.aruco.drawAxis(current_frame, self.camera_matrix, self.distortion_coefficients, rvec, tvec, (MARKER_SIZE_MM / 1000)*0.5)
                
        
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
