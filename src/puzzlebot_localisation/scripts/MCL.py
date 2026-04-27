#!/usr/bin/env python3
#Codigo base usado para el mapa
#Se va a actualizar la logica a c++ para mayor rendimiento
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import random
from tf_transformations import quaternion_from_euler
from rclpy.qos import qos_profile_sensor_data

class MonteCarloSLAM(Node):
    def __init__(self):
        super().__init__('monte_carlo_slam_node')
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_odom = None

        self.num_particles = 50 # probar con pocas y analizar el rendimiento
        self.particles = np.zeros((self.num_particles, 4))
        self.particles[:, 3] = 1.0 / self.num_particles # Pesos iniciales
        
        self.map_res = 0.01  # 1cm por pixel
        self.map_width = 200 # metros
        self.map_height = 200
        self.map_origin_x = -(self.map_width * self.map_res) / 2
        self.map_origin_y = -(self.map_height * self.map_res) / 2
        self.grid = np.full((self.map_width, self.map_height), -1, dtype=np.int8) # -1 es desconocido
        self.grid_probs = np.zeros((self.map_width, self.map_height), dtype=np.float32) # Para mantener probabilidades de ocupación


    def odom_callback(self, msg):
        """
        PREDICTION
        Moves particles based on movement from encoders
        """
        self.last_odom = msg
        # Must change this to use the actual odometry data to move the particles
        # Particle motion simulated with some noise
        dx = msg.twist.twist.linear.x * 0.1 
        dy = msg.twist.twist.linear.y * 0.1
        da = msg.twist.twist.angular.z * 0.1

        for p in self.particles:
            p[0] += dx + random.gauss(0, 0.01) # Gaussian noise
            p[1] += dy + random.gauss(0, 0.01)
            p[2] += da + random.gauss(0, 0.005)

        

    def scan_callback(self, msg):
        """
        UPDATE AND MAPPING STAGE:
        Uses the LiDAR to adjust weights and mark the map.
        """
        if self.last_odom is None:
            self.get_logger().warning("Waiting for odometry data...")
            return
        # 1. Select particle with highest weight
        best_particle = self.particles[np.argmax(self.particles[:, 3])]

        # 2. Ray Casting for mapping: Update the map based on the best particle's pose and the LiDAR scan
        self.update_map_with_scan2(best_particle, msg)
        
        # 3. Publish the transform from 'map' to 'odom' based on the best particle's pose
        self.publish_transform(best_particle, self.last_odom)
        self.publish_map()

    def update_weights(self, scan):
        for p in self.particles:
            score = 0
            # Tomamos solo algunos puntos del scan para no saturar el CPU
            for i in range(0, len(scan.ranges), 10): 
                dist = scan.ranges[i]
                if dist > scan.range_max or dist < scan.range_min: continue
                
                angle = p[2] + scan.angle_min + (i * scan.angle_increment)
                tx = int((p[0] + dist * np.cos(angle) - self.map_origin_x) / self.map_res)
                ty = int((p[1] + dist * np.sin(angle) - self.map_origin_y) / self.map_res)
                
                if 0 <= tx < self.map_width and 0 <= ty < self.map_height:
                    # Si la partícula "ve" una pared donde el mapa ya dice que hay una pared, ¡premio!
                    if self.grid[tx, ty] == 100:
                        score += 1
            
            p[3] = score # Actualizamos el peso
        
        # Normalizar pesos
        sum_w = np.sum(self.particles[:, 3])
        if sum_w > 0:
            self.particles[:, 3] /= sum_w

    def update_map_with_scan(self, pose, scan):
        #Convert LiDAR readings from polar to Cartesian coordinates relative to the map
        #and update the cells in self.grid using the particle's pose
        """
        pose: [x, y, theta, weight] from best particle
        scan: LaserScan message
        """
        ox, oy, otheta = pose[:3]
        
        # Transform robot position from world coordinates to grid indices
        start_x = int((ox - self.map_origin_x) / self.map_res)
        start_y = int((oy - self.map_origin_y) / self.map_res)

        for i, dist in enumerate(scan.ranges):
            # Ignore noisy and out of range readings
            if dist > scan.range_max or dist < scan.range_min:
                continue
                
            # Global angle of the LiDAR ray in the world frame
            angle = otheta + scan.angle_min + (i * scan.angle_increment)
            
            # Impact position from the robot's pose to world
            end_x_m = ox + dist * np.cos(angle)
            end_y_m = oy + dist * np.sin(angle)
            
            # Impact position in grid cells
            end_x = int((end_x_m - self.map_origin_x) / self.map_res)
            end_y = int((end_y_m - self.map_origin_y) / self.map_res)

            # 1. Mark the impact point as occupied (100)
            if 0 <= end_x < self.map_width and 0 <= end_y < self.map_height:
                self.grid[end_x, end_y] = 100

            # 2. (Opcional) Trazar línea de celdas libres entre start y end
            # Esto es lo que permite que el mapa "limpie" obstáculos que se movieron
            # Se puede usar una implementación simple de la línea de Bresenham aquí

    # def update_map_with_scan2(self, pose, scan):
    #     ox, oy, otheta = pose[:3]
    #     start_x = int((ox - self.map_origin_x) / self.map_res)
    #     start_y = int((oy - self.map_origin_y) / self.map_res)

    #     for i, dist in enumerate(scan.ranges):
    #         if dist > scan.range_max or dist < scan.range_min:
    #             continue
                
    #         angle = otheta + scan.angle_min + (i * scan.angle_increment)
    #         end_x = int((ox + dist * np.cos(angle) - self.map_origin_x) / self.map_res)
    #         end_y = int((oy + dist * np.sin(angle) - self.map_origin_y) / self.map_res)

    #         # Obtain all ray cells
    #         ray_cells = self.get_line_cells(start_x, start_y, end_x, end_y)
            
    #         for cell_x, cell_y in ray_cells[:-1]: # Todas menos la última son LIBRES
    #             if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
    #                 self.grid[cell_x, cell_y] = 0
            
    #         # Last cell is occupied
    #         if 0 <= end_x < self.map_width and 0 <= end_y < self.map_height:
    #             self.grid[end_x, end_y] = 100
    def update_map_with_scan2(self, pose, scan):
        ox, oy, otheta = pose[:3]
        start_x = int((ox - self.map_origin_x) / self.map_res)
        start_y = int((oy - self.map_origin_y) / self.map_res)

        for i, dist in enumerate(scan.ranges):
            if dist > scan.range_max or dist < scan.range_min or np.isnan(dist):
                continue
                
            angle = otheta + scan.angle_min + (i * scan.angle_increment)
            end_x = int((ox + dist * np.cos(angle) - self.map_origin_x) / self.map_res)
            end_y = int((oy + dist * np.sin(angle) - self.map_origin_y) / self.map_res)

            # 1. Obtener todas las celdas por las que pasa el rayo
            ray_cells = self.get_line_cells(start_x, start_y, end_x, end_y)
            
            # 2. "Limpiar" el camino (reducir probabilidad de ocupación)
            for cell_x, cell_y in ray_cells[:-1]:
                if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
                    # Restamos 2 (puedes tunear este valor)
                    if self.grid_probs[cell_x, cell_y] > -100:
                        self.grid_probs[cell_x, cell_y] -= 2
            
            # 3. Marcar el impacto (aumentar probabilidad)
            if 0 <= end_x < self.map_width and 0 <= end_y < self.map_height:
                if self.grid_probs[end_x, end_y] < 100:
                    self.grid_probs[end_x, end_y] += 5 # Más peso al impacto

        # 4. Convertir las probabilidades al formato de OccupancyGrid (-1, 0, 100)
        # Solo marcamos como ocupado si la probabilidad acumulada es alta (> 10 por ejemplo)
        self.grid = np.full((self.map_width, self.map_height), -1, dtype=np.int8)
        self.grid[self.grid_probs < -5] = 0
        self.grid[self.grid_probs > 10] = 100    


    def get_line_cells(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get cells between two points."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def save_map_as_image(self):
        # Transform grid (-1 to 100 ) to image format (0 to 255)
        image_data = np.copy(self.grid)
        image_data[image_data == -1] = 127  # Unknown -> Gray
        image_data[image_data == 100] = 0   # Occupied -> Black
        image_data[image_data == 0] = 255   # Free -> White
        
        cv2.imwrite('mapa_test2.png', image_data)
        self.get_logger().info("Mapa generado")

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.map_res
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def publish_transform(self, best_particle_pose, odom_msg):
        """
        Calculates and publishes the TF from 'map' to 'odom'
        best_particle_pose: [x, y, theta]
        odom_msg: Last messsage received
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
    
        # Pose_map_to_odom = Pose_map_to_base * inv(Pose_odom_to_base)
        # Practical simplification:
        t.transform.translation.x = best_particle_pose[0] - odom_msg.pose.pose.position.x
        t.transform.translation.y = best_particle_pose[1] - odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # Rotation simplified for 2D
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, best_particle_pose[2] - self.get_yaw_from_odom(odom_msg))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def get_yaw_from_odom(self, msg):
        """Extracts the yaw angle from an odometry quaternion."""
        q = msg.pose.pose.orientation
        # Simple formula for converting quaternion to Euler angles (Yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = MonteCarloSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_map_as_image() # Guardar al salir
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()