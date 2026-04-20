#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
import numpy as np


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Parámetros predeterminados basados en la resolución del video
        self.declare_parameter('frame_width', 2160)
        self.declare_parameter('frame_height', 3840)
        self.declare_parameter('camera_name', 'camera')
        
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.camera_name = self.get_parameter('camera_name').value
        
        # Crear publicador de CameraInfo
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 
            'camera_info', 
            10
        )
        
        # Crear servicio para set_camera_info (dummy)
        self.set_camera_info_srv = self.create_service(
            SetCameraInfo,
            'set_camera_info',
            self.set_camera_info_callback
        )
        
        # Timer para publicar camera_info periódicamente
        self.timer = self.create_timer(0.1, self.publish_camera_info)
        
        self.get_logger().info(f'Camera Info Publisher initialized for {self.frame_width}x{self.frame_height}')
    
    def publish_camera_info(self):
        """Publique la información de la cámara"""
        msg = CameraInfo()
        msg.header.frame_id = 'camera'
        msg.height = self.frame_height
        msg.width = self.frame_width
        
        # Matriz de calibración predeterminada (identidad escalada)
        focal_length = self.frame_width
        msg.k = [
            focal_length, 0, self.frame_width / 2,
            0, focal_length, self.frame_height / 2,
            0, 0, 1
        ]
        
        # Matriz de proyección
        msg.p = [
            focal_length, 0, self.frame_width / 2, 0,
            0, focal_length, self.frame_height / 2, 0,
            0, 0, 1, 0
        ]
        
        # Matriz de distorsión (sin distorsión)
        msg.d = [0, 0, 0, 0, 0]
        
        # Matriz de rectificación (identidad)
        msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        
        msg.distortion_model = 'plumb_bob'
        
        self.camera_info_pub.publish(msg)
    
    def set_camera_info_callback(self, request, response):
        """Callback para el servicio set_camera_info (dummy)"""
        self.get_logger().info('set_camera_info service called (dummy implementation)')
        response.success = True
        response.status_message = "Camera info set"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
