import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math

class PuzzleBotOdom(Node):
    def __init__(self):
        super().__init__('odometry_node')
        ns = self.get_namespace().strip('/')
        self.frame_prefix = ns + '/' if ns else ''
        # Parámetros físicos (igual que tu URDF/plugin)
        self.R = 0.05    # wheel_radius
        self.L = 0.18    # robot_width

        # Estado del robot
        self.x     = 0.0
        self.y     = 0.0
        self.yaw   = 0.0

        # Velocidades de ruedas (rad/s desde encoders)
        self.wr = 0.0
        self.wl = 0.0

        # Ángulos acumulados para joint_states
        self.theta_r = 0.0
        self.theta_l = 0.0

        self.last_time = None

        # Subscribers a encoders reales de Gazebo
        self.create_subscription(Float32, '/VelocityEncR', self.cb_r, 10)
        self.create_subscription(Float32, '/VelocityEncL', self.cb_l, 10)

        # TF broadcaster
        self.tf_br = TransformBroadcaster(self)

        # Publisher de joint_states (para robot_state_publisher)
        # Relative topic (no leading '/') so it respects the node's namespace
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_timer(0.001, self.update)  # 100 Hz

    def cb_r(self, msg): self.wr = msg.data
    def cb_l(self, msg): self.wl = msg.data

    def update(self):
        now = self.get_clock().now()
        t   = now.nanoseconds / 1e9

        if self.last_time is None:
            self.last_time = t
            return

        dt = t - self.last_time
        self.last_time = t

        if dt <= 0.0:
            return

        # Velocidades lineales de cada rueda
        vr = self.wr * self.R
        vl = self.wl * self.R

        # Velocidad del robot
        v = (vr + vl) / 2.0
        w = (vr - vl) / self.L

        # Integrar posición
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # Integrar ángulos de ruedas
        self.theta_r += self.wr * dt
        self.theta_l += self.wl * dt

        current_time = now.to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # --- TF odom → base_link ---
        tf = TransformStamped()
        tf.header.stamp    = current_time
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = self.frame_prefix + 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

        # --- JointStates reales ---
        js = JointState()
        js.header.stamp = current_time
        js.name     = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.theta_l, self.theta_r]
        js.velocity = [self.wl, self.wr]
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PuzzleBotOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()