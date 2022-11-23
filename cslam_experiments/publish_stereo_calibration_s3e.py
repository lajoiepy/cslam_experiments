#!/usr/bin/env python3
# Loop Closure Detection service
# Abstraction to support multiple implementations of loop closure detection for benchmarking

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image, CameraInfo
import yaml

class CalibrationPublisher(Node):
    """ Publish stereo calibration """

    def __init__(self):
        """Initialization and parameter parsing"""
        super().__init__('loop_closure_detection')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_id', None),
                        ('frontend.stereo_calibration_file', None),
                        ])
        self.params = {}
        self.params['robot_id'] = self.get_parameter('robot_id').value
        self.params["frontend.stereo_calibration_file"] = self.get_parameter(
            'frontend.stereo_calibration_file').value

        # Subscribe to the stereo images
        self.sub_left = self.create_subscription(
            Image, '/r{}/stereo_camera/left/image_raw'.format(self.params['robot_id']), self.left_callback, 10)
        self.sub_right = self.create_subscription(  
            Image, '/r{}/stereo_camera/right/image_raw'.format(self.params['robot_id']), self.right_callback, 10)

        # Publish the stereo calibration
        self.pub_left = self.create_publisher(CameraInfo, '/r{}/stereo_camera/left/camera_info'.format(self.params['robot_id']), 10)
        self.pub_right = self.create_publisher(CameraInfo, '/r{}/stereo_camera/right/camera_info'.format(self.params['robot_id']), 10)

        # Read the calibration file (yaml)
        self.calib_info = yaml.load(open(self.params["frontend.stereo_calibration_file"], 'r'), Loader=yaml.FullLoader)
        self.get_logger().info("Loaded calibration file: {}".format(self.params["frontend.stereo_calibration_file"]))
        self.left_calibration = CameraInfo()
        self.right_calibration = CameraInfo()
        self.left_calibration.height = self.calib_info['LEFT.height']
        self.left_calibration.width = self.calib_info['LEFT.width']
        self.left_calibration.distortion_model = "plumb_bob"
        self.left_calibration.d = self.calib_info['LEFT.D']['data']
        self.left_calibration.k = self.calib_info['LEFT.K']['data']
        self.left_calibration.r = self.calib_info['LEFT.R']['data']
        self.left_calibration.p = self.calib_info['LEFT.P']['data']

        self.right_calibration.height = self.calib_info['RIGHT.height']
        self.right_calibration.width = self.calib_info['RIGHT.width']
        self.right_calibration.distortion_model = "plumb_bob"
        self.right_calibration.d = self.calib_info['RIGHT.D']['data']
        self.right_calibration.k = self.calib_info['RIGHT.K']['data']
        self.right_calibration.r = self.calib_info['RIGHT.R']['data']
        self.right_calibration.p = self.calib_info['RIGHT.P']['data']

    def left_callback(self, msg):
        """Publish the calibration"""
        self.left_calibration.header = msg.header
        self.pub_left.publish(self.left_calibration)

    def right_callback(self, msg):
        """Publish the calibration"""
        self.right_calibration.header = msg.header
        self.pub_right.publish(self.right_calibration)

if __name__ == '__main__':

    rclpy.init(args=None)
    cp = CalibrationPublisher()
    cp.get_logger().info('Initialization done.')
    rclpy.spin(cp)
    rclpy.shutdown()
