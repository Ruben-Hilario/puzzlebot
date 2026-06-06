#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
import os
import time
import numpy as np
from ament_index_python import get_package_share_directory
import yaml
from rclpy.qos import qos_profile_sensor_data


class Utils(Node):
    def __init__(self):
        super().__init__('yolo_metrics_node')
        try: 
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('Ultralytics YOLO library not found"')
            raise
        model_path = os.path.join(get_package_share_directory('puzzlebot_vision'),'models','fine_tunned.pt')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/video_frames', 
            self.image_callback,
            10
        )
        
        self.class_ID = 3
        self.model_name = "yolov26n"
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.MAX_SPEED = 0.05

        self.publisher = self.create_publisher(Image, '/annotated_yolo', 10)        
        self.al_pub = self.create_publisher(Image, '/align', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Alignment testing')

    def image_callback(self, msg):
        try:
            twist_msg = Twist()
            largest_area = 0
            best_box = None
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            h_center, w_center = h//2, w//2
            
            results = self.model(cv_image, device='cpu', verbose=False)
            annotated_frame = results[0].plot()
            alignment_frame = annotated_frame.copy()
            cv2.line(alignment_frame, (w_center, 0), (w_center, h), (0, 255, 0), 2)
            
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                if cls_id == self.class_ID:
                    coords = box.xyxy[0].cpu().numpy().astype(int)
                    xmin, ymin, xmax, ymax = coords
                    area = (xmax - xmin) * (ymax - ymin)
                    if area > largest_area:
                        largest_area = area
                        best_box = [xmin, ymin, xmax, ymax]
            if best_box is not None:
                center_x = (best_box[0] + best_box[2]) // 2
                center_y = (best_box[1] + best_box[3]) // 2
                cv2.circle(alignment_frame, (center_x, center_y), 8, (0, 0, 255), -1)

                error_x = float(w_center - center_x) / (w / 2.0)
                error_y = float(h_center - center_y) / (h / 2.0)
                Kp = 0.1
                Kp_ = 0.1
                ang_z = error_x * Kp
                linear = error_y * Kp_
                twist_msg.linear.x = np.clip(linear, -self.MAX_SPEED, self.MAX_SPEED)
                twist_msg.angular.z = np.clip(ang_z, -self.MAX_SPEED, self.MAX_SPEED)
                
                self.publish(annotated_frame, alignment_frame)
            else:
                self.get_logger().info(f'Target class {self.class_ID} not found in this frame.')
                annotated_frame = results[0].plot()
                out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                self.publisher.publish(out_msg)
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
        
    def publish(self, annotated, alignment):
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        al_msg = self.bridge.cv2_to_imgmsg(alignment, encoding='bgr8')
        self.publisher.publish(out_msg)
        self.al_pub.publish(al_msg)

class PoseNode(Node):
    def __init__(self):
        super().__init__('yolo_metrics_node')
        try: 
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('Ultralytics YOLO library not found"')
            raise
        model_path = os.path.join(get_package_share_directory('puzzlebot_vision'),'models','fine_tunned.pt')
        yaml_path = os.path.join(get_package_share_directory('puzzlebot_localisation'),'config','path.yaml')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/video_frames', 
            self.image_callback,
            10
        )
        self.state_sub = self.create_subscription(String, '/vuelta',state_cb,10) #receive vuelta_1 or vuelta_2 depending on the situationship
        
        self.class_ID = 3
        self.model_name = "yolov26n"
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.MAX_SPEED = 0.05

        self.waypoints = self.load_yaml()
        
        self.publisher = self.create_publisher(Image, '/annotated_yolo', 10)        
        self.al_pub = self.create_publisher(Image, '/align', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose',qos_profile_sensor_data)
        self.get_logger().info('Alignment testing')
        
    def load_yaml(self):    
        with open("yaml_path","r") as file:
            return yaml.safe_load(file)

    def state_cb(self,msg):
        
        
    """
    Vuelta 1:
        1. Waypoint
        2. Rotar izquierda a derecha hasta encontrar el qr. 
            - Static turn
            - Crop vertical from bounding box width y ahi encontrar el qr
        3. Ir a waypoint dedicado
            - Localizarse en waypoint
            - Decode QR
            - Comenzar alignment function
            - Levantar montacargas
        4. Acceder montacargas
        5. Back
        6. Bajar montacargas
        7. Switch to montecarlo nuevamente.
        8. Ir a waypoint de decode
    Vuelta 2:
        1. Waypoint central.
        2. Rotar izquierda a derecha hasta encontrar el qr.
            - revisar el rack izquierdo con la rotacion
            - Rotar a inicio del segundo derecho.
            - revisar el rack derecho con la rotacion,
            - Rotar a inicio del rack aislado
            - revisar el rack aislado con la rotacion
            (
                ~ Turn
                ~ Crop vertical de bounding box width y encontrar el qr
            )
        3. Ir a waypoint dedicado
            - Localizarse en waypoint
            - Decode QR
            - Comenzar alignment function
            - Levantar montacargas
        4. Acceder a montacargas
        5. Back
        6. Bajar montacargas
        7. Switch to montecarlo nuevamente
        8. Ir a waypoint de decode
    """ 

        
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
