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
import psutil
import csv

class YOLOMetrics(Node):
    def __init__(self):
        super().__init__('yolo_metrics_node')
        try: 
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('Ultralytics YOLO library not found"')
            raise
        
        # 1. Load YOLO model (it will automatically use the GPU if ROCm is set up)
        # Using yolov8n.pt for speed; it will download on first run
        model_path = os.path.join('/home/rocm_ws/ros2_ws/src/puzzlebot_vision/models', 'yoloN_best.pt')
        self.model = YOLO(model_path)
        
        # 2. Bridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # 3. Subscriber: Listen to the video stream from your Ubuntu 22 container
        self.subscription = self.create_subscription(
            Image,
            '/video_frames', 
            #self.image_callback,
            self.image_cb,
            10
        )
        
        #
        self.model_name = "yolov8n"
        self.csv_filename = f'metrics_{self.model_name}.csv'
        self.process = psutil.Process(os.getpid())
        with open(self.csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'latency_ms', 'cpu_percent', 'ram_mb', 'fps'])

        # 4. Publisher: Send the results back out (optional)
        self.publisher = self.create_publisher(Image, '/yolo/visual_result', 10)
        
        self.get_logger().info('YOLO Inference Node started on Ubuntu 24.04 (Jazzy)')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run Inference
            # device=0 tells it to use the first AMD GPU found by ROCm
            results = self.model(cv_image, device=0, verbose=False)

            # Visualize results on the frame
            annotated_frame = results[0].plot()

            # Convert back to ROS message and publish
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')

    def image_cb(self, msg):
        start_time = time.perf_counter()
        
        try:
            # --- Proceso de Inferencia ---
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, device=0, verbose=False)
            
            # --- Cálculos ---
            end_time = time.perf_counter()
            latency = (end_time - start_time) * 1000
            fps = 1.0 / (end_time - start_time)
            cpu = self.process.cpu_percent()
            ram = self.process.memory_info().rss / (1024 * 1024)
            ts = time.time()

            # --- Guardar en CSV ---
            with open(self.csv_filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([ts, latency, cpu, ram, fps])

            annotated_frame = results[0].plot()
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.publisher.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.get_logger().error(f'Error: {e}')


class RFDETRMetrics(Node):
    def __init__(self):
        super().__init__('rf_detr_metrics_node')
        try:
            from rfdetr import RFDETRNano
            import supervision as sv
        except ImportError:
            self.get_logger().error('Required libraries not found. Please install "rfdetr" and "supervision" with pip.')
            raise
        # 1. Load RFDETR model (it will automatically use the GPU if ROCm is set up)
        model_path = os.path.join('/home/rocm_ws/ros2_ws/src/puzzlebot_vision/models', 'rfdetr1.pth')
        self.model = RFDETRNano(pretrain_weights=model_path)
        self.model_name = "rfdetr_nano"
        self.csv_filename = f'metrics_{self.model_name}.csv'
        self.process = psutil.Process(os.getpid())
        self.mask_annotator = sv.MaskAnnotator()
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.classes = ["puxxlebot2","ELURIEL","PALET","TRUCK"]

        # 2. Bridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # 3. Subscriber: Listen to the video stream from your Ubuntu 22 container
        self.subscription = self.create_subscription(
            Image,
            '/video_frames', 
            self.image_cb,
            10
        )
        
        # 4. Publisher: Send the results back out (optional)
        self.publisher = self.create_publisher(Image, '/rfdetr/visual_result', 10)
        with open(self.csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'latency_ms', 'cpu_percent', 'ram_mb', 'fps'])

        self.get_logger().info('RFDETR Inference Node started on Ubuntu 24.04 (Jazzy)')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h_c, w_c = cv_img.shape[:2]
            dets = self.model.predict(cv_img, threshold=0.65)
            dets.labels = [
                f"{self.classes[cid] if cid < len(self.classes) else f'cls_{cid}'} {score:.2f}"
                for cid, score in zip(dets.class_id, dets.confidence)
            ]

            det_img = self.box_annotator.annotate(cv_img.copy(), dets)
            det_img = self.label_annotator.annotate(det_img, dets, labels=dets.labels)
            
            self.publisher.publish(self.bridge.cv2_to_imgmsg(det_img, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
    def image_cb(self, msg):
        start_time = time.perf_counter()
        try:
            # 1. Preparación e Inferencia
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Medir solo la inferencia pura si lo deseas, 
            # pero aquí medimos el ciclo completo del callback
            dets = self.model.predict(cv_img, threshold=0.65)
            
            # 2. Post-procesamiento y Anotación
            dets.labels = [
                f"{self.classes[cid] if cid < len(self.classes) else f'cls_{cid}'} {score:.2f}"
                for cid, score in zip(dets.class_id, dets.confidence)
            ]
            det_img = self.box_annotator.annotate(cv_img.copy(), dets)
            det_img = self.label_annotator.annotate(det_img, dets, labels=dets.labels)
            
            # 3. Publicación
            self.publisher.publish(self.bridge.cv2_to_imgmsg(det_img, encoding='bgr8'))

            # --- Cálculo de Métricas ---
            end_time = time.perf_counter()
            duration = end_time - start_time
            
            latency_ms = duration * 1000
            fps = 1.0 / duration if duration > 0 else 0
            cpu = self.process.cpu_percent()
            ram = self.process.memory_info().rss / (1024 * 1024)
            
            # Guardar en CSV
            with open(self.csv_filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([time.time(), latency_ms, cpu, ram, fps])

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
        

def main(args=None):
    rclpy.init(args=args)
    node = YOLOMetrics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
