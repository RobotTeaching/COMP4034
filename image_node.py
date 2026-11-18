import rclpy
import math
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_pose, PoseStamped
from scipy.spatial.transform import Rotation
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
        self.reference_frame = 'camera_rgb_optical_frame' # We need to transform from the camera..
        self.target_frame = 'base_link'      # to the base of the robot!
        
        # Create a subscriber for the Compressed Images
        self.subscription_image = self.create_subscription(
            CompressedImage,
            '/camera_depth/image_raw/compressed', # !! This topic is for the SIMULATED robot. You'll need to change it for real life
            self.image_callback,
            qos_profile_sensor_data)
        
        # Create a subscriber for the Camera Info (We need this for pose estimation)
        self.subscription_camerainfo = self.create_subscription(
                    CameraInfo,
                    '/camera_depth/camera_info', # !! This topic is for the SIMULATED robot. You'll need to change it for real life
                    self.camera_info_callback,
                    qos_profile_sensor_data)
        
        self.camera_matrix = None
        self.distortion_coefficients = None
        
        self.image = []

        # In this code, we use a timer to process the images every 0.5s
        # This is OK, but there may be a better way...
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def image_callback(self, msg: CompressedImage):
        # self.get_logger().info('Receiving Compressed Image Frame')
        
        # Logic to convert Compressed Images to CV2
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
        if frame is None: # Sometimes the decoding may fail, hence we catch the error here
            self.get_logger().info('Failed to decode image')
            return
        
        self.image = frame # Store the latest Image
        
        
    def camera_info_callback(self, msg: CameraInfo):
        #self.get_logger().info('Receiving camera info')
    
        # Here we are storing the camera intrinsics.
        # You don't need to understand exactly what these are or how they work
        # The only thing you need to know is that we need this to calculat the pose of the aruco,
        #  and they are unique for every camera
    
        self.camera_matrix = np.array(msg.k).reshape((3,3)) # (Camera Intrinsic Matrix)
        self.distortion_coefficients = np.array(msg.d) # (Lens Distorion)
        
    def timer_callback(self):
        # Here is where our processing is currently happening
        
        current_frame = self.image
        if current_frame == []:
            return

        # We use cv2 to detect the ARUCO markers (if any) in the image.
        # `corners` : the pixel values where the corners of the markers are
        # `marker_ids` : the id value of the detect markers
        corners, marker_ids, _ = cv2.aruco.detectMarkers(current_frame, ARUCO_DICT)
        
        
        # This function just draws on the image so we can display the detections
        current_frame = cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
        
        try:
            #Find the transform that ROS2 provides (base from cam)
            cam_to_base = self.tf_buffer.lookup_transform(self.target_frame,
                                                          self.reference_frame,
                                                          rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Failed to get transform between {self.target_frame} and {self.reference_frame}: {ex}')
            return
        
        
        if marker_ids is not None and self.camera_matrix is not None:
            # Now with the corners of our markers, and the camera info, we can calculate the pose of the marker
            for i in range(0, len(marker_ids)):
                rvec, tvec, markerpoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE_MM / 1000, self.camera_matrix, self.distortion_coefficients)
                # rvec is the ROTATION vector, tvec is the TRANSFORMATION vector
                
                translation = tvec.flatten()
                rotation = rvec.ravel()

                # Wrapping the pose information (`do_transform` expects a PoseStamped)
                mpose = PoseStamped()
                mpose.header.frame_id = self.reference_frame
                mpose.header.stamp = self.get_clock().now().to_msg()
                mpose.pose.position.x =  translation[0]
                mpose.pose.position.y =  translation[1]
                mpose.pose.position.z =  translation[2]
                
                rot_m, _ = cv2.Rodrigues(rvec)
                quat = Rotation.from_matrix(rot_m).as_quat()
                
                mpose.pose.orientation.x = quat[0]
                mpose.pose.orientation.y = quat[1]
                mpose.pose.orientation.z = quat[2]
                mpose.pose.orientation.w = quat[3]
                
                
                # Perform the transformation
                marker_in_robot_frame = do_transform_pose(mpose.pose, cam_to_base)
                marker_coords = marker_in_robot_frame.position
                
                # These are the coordinates relative to the center of the robot.
                # If you want these to be relative to odom, simply sum them!
                x,y,z = marker_coords.x, marker_coords.y, marker_coords.z
                
                # Print out some details
                self.get_logger().info(f"Detected Aruco with ID {marker_ids[i]}")
                self.get_logger().info(f"This Marker has coordinates {x} (forward),{y} (left), {z} (up)")
                # Draw the frame of the marker (its local axes) on the image
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
