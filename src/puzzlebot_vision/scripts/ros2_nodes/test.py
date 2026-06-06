#!/usr/bin/env python3

#ROS2 node to test out model inference
#The node is made in a way to do either inference or test models with local media, adjust the mode in the launch file "inference.launch.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import numpy as np
from ament_index_python import get_package_share_directory

class Utils():
    pass

class PoseNode(Node):
    def __init__(self):
        super().__init__('yolo_metrics_node')
        try: 
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('Ultralytics YOLO library not found"')
            raise
        #model_path = os.path.join('/home/rocm_ws/ros2_ws/src/puzzlebot_vision/models', 'final_yolo.pt')
        model_path = os.path.join(get_package_share_directory('puzzlebot_vision'),'models','final_yolo.pt')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/video_frames', 
            # self.image_callback,
            self.image_cb,
            10
        )
        
        self.class_ID = 3
        self.model_name = "yolov8n"
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])

        self.publisher = self.create_publisher(Image, '/yolo/video', 10)
        self.hsv = self.create_publisher(Image,'/hsv',10)
        self.canny = self.create_publisher(Image,'/canny',10)
        self.lines = self.create_publisher(Image,'lines',10)
        
        
        self.get_logger().info('YOLO Inference Node started on Ubuntu 24.04 (Jazzy)')

    def image_callback(self, msg):
        utils = Utils()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, device='cpu', verbose=False)
            annotated_frame = results[0].plot()
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.publisher.publish(out_msg)
            
            utils.roi()
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
            
    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, device='cpu', verbose=False)
            
            TARGET_CLASS_ID = 3
            
            largest_area = 0
            best_box = None
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                if cls_id == TARGET_CLASS_ID:
                    coords = box.xyxy[0].cpu().numpy().astype(int)
                    xmin, ymin, xmax, ymax = coords
                    area = (xmax - xmin) * (ymax - ymin)
                    if area > largest_area:
                        largest_area = area
                        best_box = [xmin, ymin, xmax, ymax]
            
            if best_box is not None:
                xmin, ymin, xmax, ymax = best_box
                cropped_roi = cv_image[max(0, ymin):ymax, max(0, xmin):xmax]
                blue_mask,hsv_roi = self.blue_process(cropped_roi)
                canny_edges = cv2.Canny(blue_mask, 30, 70)
                bgr_canny = cv2.cvtColor(canny_edges, cv2.COLOR_GRAY2BGR)
                lines = cv2.HoughLinesP(canny_edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)

                an_lines = self.draw_lines(lines,  bgr_canny.copy())
                self.publish(cropped_roi, bgr_canny, an_lines,hsv_roi)
            else:
                self.get_logger().info(f'Target class {TARGET_CLASS_ID} not found in this frame.')
                annotated_frame = results[0].plot()
                out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Inference or Cropping failed: {e}')

    def publish(self,cropped_roi,bgr_canny, lines,hsv):
        out_msg = self.bridge.cv2_to_imgmsg(cropped_roi, encoding='bgr8')
        canny_msg = self.bridge.cv2_to_imgmsg(bgr_canny,encoding='bgr8')
        l_msg = self.bridge.cv2_to_imgmsg(lines, encoding='bgr8')
        hsv_m = self.bridge.cv2_to_imgmsg(hsv,encoding='bgr8')
        self.publisher.publish(out_msg)
        self.canny.publish(canny_msg)
        self.lines.publish(l_msg)
        self.hsv.publish(hsv_m)
        
    def draw_lines(self, lines,line_image):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1
                extension_factor = 50 
                length = np.sqrt(dx**2 + dy**2)
                if length == 0: continue # Avoid division by zero
                
                ux = dx / length
                uy = dy / length
                
                # Calculate new, expanded endpoints
                ex1 = int(x1 - ux * extension_factor)
                ey1 = int(y1 - uy * extension_factor)
                ex2 = int(x2 + ux * extension_factor)
                ey2 = int(y2 + uy * extension_factor)
                cv2.line(line_image, (ex1, ey1), (ex2, ey2), (0, 255, 0), 3)
        return line_image
        
    def blue_process(self, cropped_roi):
        hsv_roi = cv2.cvtColor(cropped_roi, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_roi, self.lower_blue, self.upper_blue)
        blue_mask = cv2.GaussianBlur(blue_mask, (5, 5), 0)
        return blue_mask, hsv_roi
        

def main(args=None):
    rclpy.init(args=args)
    try:
        node=PoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
