import rclpy
from rclpy.node import Node
import numpy as np

class KalmanFilter():
    def __init__(self):
        super.__init__('kalman_node')
        self.timer = self.create_timer(1.0,self.main_loop)
        self.mu = None
        self.u = None
        self.delta_t = None
        self.E = None

    def timer_callbacl(self):
        pass

    def model_linearization(self):
        x = self.mu[0]
        y=self.mu[1]
        theta= self.mu[2]
        v=self.u[0]
        w=self.u[1]
        #transition function
        tF = np.array([                 
            x+v*np.cos(theta)*delta_t,
            y+v*np.sin(theta)*delta_t,
            theta+w*delta_t
        ])
        #jacobian
        J = np.array([
            [1,0,-v*np.sin(theta)*delta_t],
            [0,1,v*np.cos(theta)*delta_t],
            [0,0,1]
        ])
        return g, J
    
    def odometry_observation(self, mu):
        return mu, np.eye(3)

    def 

class EKF():
    def __init__(self):
        super.__init__('kalman_node')
        self.mu = None
        self.u = None
        self.delta_t = None
        self.E = None
        self.g = None
    def 
    


def main(args=None):
    rclpy.init(args=args)
    kalman_node = KalmanFilter()
    rclpy.spin(kalman_node)
    kalman_node.destroy_node()
    rclpy.shutdown

if __name__=="__main__":
    main()