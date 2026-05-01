#!/usr/bin/env python3
#ROS2 node to test out model inference
#The node is made in a way to do either inference or test models with local media, adjust the mode in the launch file "inference.launch.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import time

class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
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
            self.image_callback,
            10
        )
        
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


class RFDERTInferenceNode(Node):
    def __init__(self):
        super().__init__('rf_detr_inference_node')
        try:
            from rfdetr import RFDETRNano
            import supervision as sv
        except ImportError:
            self.get_logger().error('Required libraries not found. Please install "rfdetr" and "supervision" with pip.')
            raise
        # 1. Load RFDETR model (it will automatically use the GPU if ROCm is set up)
        model_path = os.path.join('/home/rocm_ws/ros2_ws/src/puzzlebot_vision/models', 'rfdetr1.pth')
        self.model = RFDETRNano(pretrain_weights=model_path)
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
            self.image_callback,
            10
        )
        
        # 4. Publisher: Send the results back out (optional)
        self.publisher = self.create_publisher(Image, '/rfdetr/visual_result', 10)
        
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
    

class YOLODAMOInferenceNode(Node):
    def __init__(self):
        super().__init__('yolodamo_inference_node')
        try:
            import onnxruntime as ort
            import numpy as np
        except ImportError:
            self.get_logger().error('Required libraries not found. Please install "onnxruntime" and "numpy" with pip.')
            raise

        self.ort = ort
        self.np = np
        self.model_path = os.path.join(
            '/home/rocm_ws/ros2_ws/src/puzzlebot_vision/models',
            'damoyolo_tinynasL18_Nm.onnx')
        self.session = self._create_session(self.model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]

        self.class_names = ['ELURIEL', 'PALET', 'TRUCK']
        self.conf_threshold = 0.03
        self.iou_threshold = 0.65
        self.image_size = 640
        self.strides = [8, 16, 32]
        self.reg_max = 7
        self.num_classes = len(self.class_names)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/video_frames',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/yolodamo/visual_result', 10)
        self.get_logger().info('YOLODamo ONNX inference node started')

    def _create_session(self, model_path):
        providers = self.ort.get_available_providers()
        if 'ROCMExecutionProvider' in providers:
            return self.ort.InferenceSession(model_path, providers=['ROCMExecutionProvider'])
        if 'CUDAExecutionProvider' in providers:
            return self.ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])
        return self.ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

    def _preprocess(self, image):
        image_resized = cv2.resize(image, (self.image_size, self.image_size), interpolation=cv2.INTER_LINEAR).astype(self.np.float32)
        image_resized = self.np.ascontiguousarray(image_resized)
        image_resized = image_resized.transpose(2, 0, 1)
        return image_resized[None, :, :, :]

    def _sigmoid(self, x):
        return 1.0 / (1.0 + self.np.exp(-x))

    def _softmax(self, x, axis=2):
        x = x - self.np.max(x, axis=axis, keepdims=True)
        exp_x = self.np.exp(x)
        return exp_x / self.np.sum(exp_x, axis=axis, keepdims=True)

    def _get_priors(self, feat_h, feat_w, stride):
        x = self.np.arange(feat_w, dtype=self.np.float32) * stride
        y = self.np.arange(feat_h, dtype=self.np.float32) * stride
        x_grid, y_grid = self.np.meshgrid(x, y)
        priors = self.np.stack(
            [x_grid.flatten(), y_grid.flatten(),
             self.np.full(x_grid.size, stride, dtype=self.np.float32),
             self.np.full(x_grid.size, stride, dtype=self.np.float32)],
            axis=-1)
        return priors

    def _decode_outputs(self, outputs):
        cls_outputs = [outputs[0], outputs[1], outputs[2]]
        reg_outputs = [outputs[3], outputs[4], outputs[5]]

        all_boxes = []
        all_scores = []

        for cls_out, reg_out, stride in zip(cls_outputs, reg_outputs, self.strides):
            cls_score = self._sigmoid(cls_out)
            _, num_classes, h, w = cls_score.shape
            cls_score = cls_score.reshape(1, num_classes, h * w).transpose(0, 2, 1)[0]

            reg_out = reg_out.reshape(1, 4, self.reg_max + 1, h, w)
            reg_out = self._softmax(reg_out, axis=2)
            reg_out = reg_out * self.np.arange(self.reg_max + 1, dtype=self.np.float32).reshape(1, 1, self.reg_max + 1, 1, 1)
            reg_out = reg_out.sum(axis=2)
            reg_out = reg_out * stride
            reg_out = reg_out.reshape(1, 4, h * w).transpose(0, 2, 1)[0]

            priors = self._get_priors(h, w, stride)
            x1 = priors[:, 0:1] - reg_out[:, 0:1]
            y1 = priors[:, 1:2] - reg_out[:, 1:2]
            x2 = priors[:, 0:1] + reg_out[:, 2:3]
            y2 = priors[:, 1:2] + reg_out[:, 3:4]
            boxes = self.np.concatenate([x1, y1, x2, y2], axis=-1)

            all_boxes.append(boxes)
            all_scores.append(cls_score)

        return self.np.concatenate(all_boxes, axis=0), self.np.concatenate(all_scores, axis=0)

    def _nms(self, boxes, scores, iou_threshold):
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = self.np.maximum(x1[i], x1[order[1:]])
            yy1 = self.np.maximum(y1[i], y1[order[1:]])
            xx2 = self.np.minimum(x2[i], x2[order[1:]])
            yy2 = self.np.minimum(y2[i], y2[order[1:]])

            w = self.np.maximum(0.0, xx2 - xx1)
            h = self.np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            inds = self.np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]

        return self.np.array(keep, dtype=self.np.int32)

    def _postprocess(self, boxes, scores):
        detections = []
        for class_id in range(self.num_classes):
            class_scores = scores[:, class_id]
            mask = class_scores > self.conf_threshold
            if not self.np.any(mask):
                continue
            filtered_boxes = boxes[mask]
            filtered_scores = class_scores[mask]
            keep = self._nms(filtered_boxes, filtered_scores, self.iou_threshold)
            for idx in keep:
                detections.append({
                    'box': filtered_boxes[idx],
                    'score': float(filtered_scores[idx]),
                    'class_id': class_id,
                    'class_name': self.class_names[class_id],
                })
        return sorted(detections, key=lambda x: x['score'], reverse=True)

    def _draw_detections(self, image, detections, scale_x, scale_y):
        for det in detections:
            x1, y1, x2, y2 = det['box']
            x1 = int(self.np.clip(x1 * scale_x, 0, image.shape[1] - 1))
            y1 = int(self.np.clip(y1 * scale_y, 0, image.shape[0] - 1))
            x2 = int(self.np.clip(x2 * scale_x, 0, image.shape[1] - 1))
            y2 = int(self.np.clip(y2 * scale_y, 0, image.shape[0] - 1))
            label = f"{det['class_name']}:{det['score']:.2f}"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return image

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h_orig, w_orig = cv_img.shape[:2]
            input_tensor = self._preprocess(cv_img)
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            boxes, scores = self._decode_outputs(outputs)
            detections = self._postprocess(boxes, scores)

            out_img = cv_img.copy()
            if detections:
                scale_x = w_orig / float(self.image_size)
                scale_y = h_orig / float(self.image_size)
                out_img = self._draw_detections(out_img, detections, scale_x, scale_y)

            self.publisher.publish(self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
            

def main(args=None):
    rclpy.init(args=args)
    node = YOLOInferenceNode()
    #node = RFDERTInferenceNode()
    #node = YOLODAMOInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
