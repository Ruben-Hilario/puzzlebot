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
from pyzbar.pyzbar import decode as zbar_decode
from std_msgs.msg import Bool


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
        self.qr_info = None

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

    def decode(self, image):
        try:
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            qr_codes = zbar_decode(gray_roi)
            
            if qr_codes:
                qr_data = qr_codes[0].data.decode('utf-8')
                return qr_data
            return None
            
        except ImportError:
            self.get_logger().error('La librería pyzbar no está instalada. Ejecuta: pip install pyzbar')
            return None
        except Exception as e:
            self.get_logger().error(f'Error al decodificar el QR: {e}')
            return None

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
class PoseNode(Node):
    def __init__(self):
        super().__init__('yolo_metrics_node')
        try: 
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('Ultralytics YOLO library not found')
            raise
        
        model_path = os.path.join(get_package_share_directory('puzzlebot_vision'), 'models', 'fine_tunned.pt')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/video_frames', 
            self.image_callback,
            10
        )
        
        self.class_ID = 3
        self.qr_class_ID = 4
        self.model_name = "yolov26n"
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.MAX_SPEED = 0.05

        self.publisher = self.create_publisher(Image, '/annotated_yolo', 10)        
        self.al_pub = self.create_publisher(Image, '/align', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Alignment testing')

        self.det_pub = self.create_publisher(Bool, '/pallet_detected', 10)
        self.qr_flag_pub = self.create_publisher(Bool, '/pallet_has_qr', 10)
        self.qr_content_pub = self.create_publisher(String, '/pallet_qr_content', 10)
        self.alineacion_pub = self.create_publisher(Bool, '/alineation/booleano', 10)

    def decode(self, roi):
        try:
            from pyzbar.pyzbar import decode as zbar_decode
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            qr_codes = zbar_decode(gray_roi)
            if qr_codes:
                return qr_codes[0].data.decode('utf-8')
            return None
        except ImportError:
            detector = cv2.QRCodeDetector()
            data, bbox, straight_qrcode = detector.detectAndDecode(roi)
            if bbox is not None and data:
                return data
            return None
        except Exception:
            return None

    def image_callback(self, msg):
        try:
            twist_msg = Twist()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            h_center, w_center = h // 2, w // 2
            
            results = self.model(cv_image, device='cpu', verbose=False)
            annotated_frame = results[0].plot()
            alignment_frame = annotated_frame.copy()
            cv2.line(alignment_frame, (w_center, 0), (w_center, h), (0, 255, 0), 2)
            
            boxes_3 = []
            boxes_4 = []
            
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                coords = box.xyxy[0].cpu().numpy().astype(int)
                if cls_id == self.class_ID:
                    boxes_3.append(coords)
                elif cls_id == self.qr_class_ID:
                    boxes_4.append(coords)
            
            best_box = None
            qr_content_string = ""
            pallet_has_qr = False
            
            # Find the specific class_ID target that actually has a QR code above it
            for b3 in boxes_3:
                xmin3, ymin3, xmax3, ymax3 = b3
                center_x3 = (xmin3 + xmax3) // 2
                
                for b4 in boxes_4:
                    xmin4, ymin4, xmax4, ymax4 = b4
                    center_x4 = (xmin4 + xmax4) // 2
                    
                    # Condition to check if QR is directly above the object
                    if ymax4 <= (ymin3 + 20) and (xmin3 <= center_x4 <= xmax3 or xmin4 <= center_x3 <= xmax4):
                        best_box = b3  # Target locked onto the element under a QR
                        
                        ymin4_safe = max(0, ymin4)
                        ymax4_safe = min(h, ymax4)
                        xmin4_safe = max(0, xmin4)
                        xmax4_safe = min(w, xmax4)
                        
                        if ymax4_safe > ymin4_safe and xmax4_safe > xmin4_safe:
                            roi = cv_image[ymin4_safe:ymax4_safe, xmin4_safe:xmax4_safe]
                            decoded_data = self.decode(roi)
                            if decoded_data is not None:
                                self.get_logger().info(f'QR Decoded: {decoded_data}')
                                qr_content_string = decoded_data
                                pallet_has_qr = True
                        break
                
                if best_box is not None:
                    break  # Stop checking other class_ID boxes once a valid pairing is discovered
            
            # Publish Statuses and Actions based on finding the specific target
            det_msg = Bool()
            qr_flag_msg = Bool()
            qr_str_msg = String()
            alineacion_msg = Bool()

            if best_box is not None:
                det_msg.data = True
                qr_flag_msg.data = pallet_has_qr
                qr_str_msg.data = qr_content_string
                alineacion_msg.data = True  # Assuming tracking status is active

                # Place visual marker circle on the targeted element
                center_x = (best_box[0] + best_box[2]) // 2
                center_y = (best_box[1] + best_box[3]) // 2
                cv2.circle(alignment_frame, (center_x, center_y), 8, (0, 0, 255), -1)

                # Compute control errors
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
                self.get_logger().info(f'Target class {self.class_ID} with QR above not found.')
                det_msg.data = False
                qr_flag_msg.data = False
                qr_str_msg.data = ""
                alineacion_msg.data = False

                annotated_frame = results[0].plot()
                out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                self.publisher.publish(out_msg)
                
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            
            # Send ROS 2 payloads
            self.det_pub.publish(det_msg)
            self.qr_flag_pub.publish(qr_flag_msg)
            self.qr_content_pub.publish(qr_str_msg)
            self.alineacion_pub.publish(alineacion_msg)
            self.vel_pub.publish(twist_msg)

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
        
    def publish(self, annotated, alignment):
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        al_msg = self.bridge.cv2_to_imgmsg(alignment, encoding='bgr8')
        self.publisher.publish(out_msg)
        self.al_pub.publish(al_msg)        
        
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseNode()
        # node = Utils()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
