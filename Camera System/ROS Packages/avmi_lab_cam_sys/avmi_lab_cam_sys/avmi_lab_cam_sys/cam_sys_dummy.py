from avmi_interfaces.srv import GetColorVals

import rclpy
from rclpy.node import Node

import cv2
import colorsys
import numpy as np


class CameraSystem(Node):
    
    def __init__(self):
        super().__init__('camera_system')
        self.srv = self.create_service(GetColorVals, 'get_color_vals', self.get_color_vals_callback)
        
    def get_color_vals_callback(self, request, response):
        
        color_vals = read_cameras()
        response.cam1 = color_vals[0]
        response.cam2 = color_vals[1]
        response.cam3 = color_vals[2]
        response.cam4 = color_vals[3]
        
        self.get_logger().info('Incoming request')
        
        return response


def read_cameras():

    color_vals = [0.0,0.5,0.5,0.0]
        
    return color_vals


def main():
    rclpy.init()
    
    camera_system = CameraSystem()
    
    rclpy.spin(camera_system)
    
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
