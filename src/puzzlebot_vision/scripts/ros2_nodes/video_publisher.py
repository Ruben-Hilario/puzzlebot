#ROS2 node to publish a video frame, useful for testing the vision pipeline without needing a physical camera or calibrating a camera
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        self.declare_parameter('video_path', '')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic_name', 'video_frames')
        self.declare_parameter('loop', True)
        
        self.video_path = self.get_parameter('video_path').value
        self.fps = self.get_parameter('fps').value
        self.topic_name = self.get_parameter('topic_name').value
        self.loop = self.get_parameter('loop').value
        
        if not self.video_path or not os.path.exists(self.video_path):
            self.get_logger().error(f'Video file not found: {self.video_path}')
            raise FileNotFoundError(f'Video file not found: {self.video_path}')
        
        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video: {self.video_path}')
            raise RuntimeError(f'Failed to open video: {self.video_path}')
        
        frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(
            f'Video: {self.video_path}\n'
            f'  Frames: {frame_count}\n'
            f'  FPS: {video_fps}\n'
            f'  Resolution: {width}x{height}'
        )
        
        self.frame_interval = 1.0 / self.fps if self.fps > 0 else 1.0 / video_fps
        
        self.timer = self.create_timer(self.frame_interval, self.publish_frame)
        self.frame_count = 0
    
    def publish_frame(self):
        """Leer y publicar el siguiente frame del video"""
        ret, frame = self.cap.read()
    
        if not ret:
            if self.loop:
                self.get_logger().info('Looping video...')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()            
                if not ret:
                    self.get_logger().error('Failed to restart video')
                    return
            else:
                self.get_logger().info('Video finished. Stopping...')
                self.timer.cancel()
                return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.frame_count += 1
            
            if self.frame_count % 100 == 0:
                self.get_logger().debug(f'Published {self.frame_count} frames')
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {e}')
    
    def destroy_node(self):
        """Limpiar recursos"""
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
