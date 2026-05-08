# !/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data

import cv2
import matplotlib.pyplot as plt
import numpy as np
from heapq import *

class GridPlannerNode(Node):
    def __init__(self):
        super().__init__('grid_planner_node')
        
        # Suscripción al mapa con QoS compatible con SLAM/Map_Server
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile_sensor_data)
            
        self.plotting_done = False
        self.start_x = 1170
        self.start_y = 1000
        self.goal_x = 1000
        self.goal_y = 1250
        
        #self.path_pub = self.create_publisher(Path, 'plan', 10)
        self.planning_done = False

        self.get_logger().info('Nodo Grid Planner iniciado. Esperando mapa...')

    def linea(self, ruta):
        xg = [p[0] for p in ruta]
        yg = [p[1] for p in ruta]
        return xg, yg

    def map_callback(self, data):
        if self.plotting_done:
            return

        self.get_logger().info(f"Recibido mapa: {data.info.width}x{data.info.height} @ {data.info.resolution:.3f}m/px")
        
        # Convertir datos del mapa a matriz numpy
        width = data.info.width
        height = data.info.height
        my_map = np.array(data.data).reshape((height, width))

        # Normalizar y filtrar obstáculos (ROS 2 map: 0 libre, 100 ocupado, -1 desconocido)
        im = np.zeros((height, width), dtype=np.float32)
        im[my_map > 0] = 1.0  # Obstáculos
        
        self.plotting_done = True
        im2 = np.flipud(im)
        im2 = im2.astype(int)

        print("\n--- Planificador de Trayectoria A* ---")
        try:
            inicio = (self.start_x, self.start_y)
            final = (self.goal_x, self.goal_y)

            print("Generando trayectoria...")
            tray = self.a_planning(im2, inicio, final)
            
            if tray:
                tray = np.asarray(tray)
                xg, yg = self.linea(tray)
                
                fig, ax = plt.subplots()
                ax.imshow(im2)
                ax.plot(yg, xg, color="white", linewidth=2)
                ax.scatter(inicio[1], inicio[0], color="green", label="Inicio")
                ax.scatter(final[1], final[0], color="red", label="Final")
                ax.legend()
                plt.title("A* Planning ROS 2")
                plt.show()
            else:
                print("No se encontró una ruta válida.")
                self.plotting_done = False # Reintentar en siguiente callback
                
        except ValueError:
            print("Entrada no válida. Por favor use números enteros.")
            self.plotting_done = False

    def a_planning(self, mapa, inicio, final):
        # --- Tu lógica de A* se mantiene igual ---
        lista_abierta = []
        lista_cerrada = []

        punto_inicial = Punto(None, inicio)
        punto_final = Punto(None, final)

        lista_abierta.append(punto_inicial)
        
        iteraciones = 0
        max_iter = 150000
        vecinos = ((0, -1), (0, 1), (-1, 0), (1, 0))

        while len(lista_abierta) > 0:
            iteraciones += 1
            
            # Buscar el punto con menor F
            punto_actual = lista_abierta[0]
            indice = 0
            for i, elemento in enumerate(lista_abierta):
                if elemento.f < punto_actual.f:
                    punto_actual = elemento
                    indice = i

            if iteraciones > max_iter:
                print("Error: Máximo de iteraciones alcanzado.")
                return self.dev_tray(punto_actual)

            lista_abierta.pop(indice)
            lista_cerrada.append(punto_actual)

            if punto_actual == punto_final:
                return self.dev_tray(punto_actual)

            hijos = []
            for nueva_pos in vecinos:
                pos_punto = (punto_actual.pos[0] + nueva_pos[0], punto_actual.pos[1] + nueva_pos[1])

                # Límites del mapa
                if pos_punto[0] >= len(mapa) or pos_punto[0] < 0 or \
                   pos_punto[1] >= len(mapa[0]) or pos_punto[1] < 0:
                    continue

                # Obstáculo
                if mapa[pos_punto[0]][pos_punto[1]] != 0:
                    continue

                nuevo_punto = Punto(punto_actual, pos_punto)
                if nuevo_punto in lista_cerrada:
                    continue

                hijos.append(nuevo_punto)

            for hijo in hijos:
                if hijo in lista_cerrada:
                    continue

                hijo.g = punto_actual.g + 1
                hijo.h = ((hijo.pos[0] - punto_final.pos[0]) ** 2) + ((hijo.pos[1] - punto_final.pos[1]) ** 2)
                hijo.f = hijo.g + hijo.h

                if any(p_abierta for p_abierta in lista_abierta if hijo == p_abierta and hijo.g > p_abierta.g):
                    continue

                lista_abierta.append(hijo)
        return None

    def dev_tray(self, punto_actual):
        tray = []
        loc = punto_actual
        while loc is not None:
            tray.append(loc.pos)
            loc = loc.padre
        return tray[::-1]

class Punto:
    def __init__(self, padre=None, pos=None):
        self.padre = padre
        self.pos = pos
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, sig):
        return self.pos == sig.pos

def main(args=None):
    rclpy.init(args=args)
    node = GridPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid, Path
# from geometry_msgs.msg import PoseStamped
# from rclpy.qos import qos_profile_sensor_data
# import numpy as np

# class GridPlannerNode(Node):
#     def __init__(self):
#         super().__init__('grid_planner_node')
        
#         # Suscripción al mapa
#         self.subscription = self.create_subscription(
#             OccupancyGrid,
#             'map',
#             self.map_callback,
#             qos_profile_sensor_data)
            
#         # Publicador para visualizar la ruta en RViz
#         self.path_pub = self.create_publisher(Path, '/plan', 10)
            
#         self.planning_done = False
        
#         # Coordenadas en PIXELES (según tu script original)
#         self.start_x = 1170
#         self.start_y = 1000
#         self.goal_x = 1000
#         self.goal_y = 1250
        
#         self.get_logger().info('Nodo Grid Planner con salida a RViz iniciado...')

#     def map_callback(self, data):
#         if self.planning_done:
#             return

#         self.get_logger().info("Mapa recibido. Calculando trayectoria...")
        
#         width = data.info.width
#         height = data.info.height
#         resolution = data.info.resolution
#         origin = data.info.origin # Pose del pixel (0,0)

#         # Convertir datos del mapa a matriz para el algoritmo
#         my_map = np.array(data.data).reshape((height, width))
#         # En ROS los mapas suelen venir con el origen abajo a la izquierda.
#         # Tu lógica original usaba flipud, la mantengo para consistencia con tu A*
#         im2 = np.flipud(my_map)
#         im2 = (im2 > 0).astype(int) # 1 si hay obstáculo, 0 si está libre

#         inicio = (self.start_x, self.start_y)
#         final = (self.goal_x, self.goal_y)

#         tray = self.a_planning(im2, inicio, final)
        
#         if tray:
#             self.get_logger().info("¡Ruta encontrada! Publicando en /plan...")
#             path_msg = Path()
#             path_msg.header.frame_id = "map" # Asegúrate de que coincida con el frame de tu RViz
#             path_msg.header.stamp = self.get_clock().now().to_msg()

#             for punto in tray:
#                 pose = PoseStamped()
#                 pose.header = path_msg.header
                
#                 # CONVERSIÓN DE PIXEL A METROS
#                 # Nota: El eje X del mapa suele ser el ancho (columnas) y el Y el alto (filas)
#                 # Como usaste flipud, invertimos la lógica de la fila para volver a coordenadas ROS
#                 pixel_y_original = height - 1 - punto[0]
#                 pixel_x_original = punto[1]

#                 pose.pose.position.x = pixel_x_original * resolution + origin.position.x
#                 pose.pose.position.y = pixel_y_original * resolution + origin.position.y
#                 pose.pose.position.z = 0.0
                
#                 path_msg.poses.append(pose)

#             self.path_pub.publish(path_msg)
#             self.planning_done = True
#         else:
#             self.get_logger().error("No se pudo encontrar una ruta válida.")

#     # --- Lógica de A* (Sin cambios significativos) ---
#     def a_planning(self, mapa, inicio, final):
#         lista_abierta = []
#         lista_cerrada = set() # Usar set para mayor velocidad de búsqueda

#         punto_inicial = Punto(None, inicio)
#         punto_final = Punto(None, final)
#         lista_abierta.append(punto_inicial)
        
#         iteracciones = 0
#         max_iter = 150000
#         vecinos = ((0, -1), (0, 1), (-1, 0), (1, 0))

#         while len(lista_abierta) > 0:
#             iteracciones += 1
            
#             # Ordenar para obtener el menor F
#             lista_abierta.sort(key=lambda x: x.f)
#             punto_actual = lista_abierta.pop(0)
#             lista_cerrada.add(punto_actual.pos)

#             if punto_actual.pos == punto_final.pos:
#                 return self.dev_tray(punto_actual)

#             if iteracciones > max_iter:
#                 return self.dev_tray(punto_actual)

#             for nueva_pos in vecinos:
#                 pos_punto = (punto_actual.pos[0] + nueva_pos[0], punto_actual.pos[1] + nueva_pos[1])

#                 if (pos_punto[0] >= len(mapa) or pos_punto[0] < 0 or 
#                     pos_punto[1] >= len(mapa[0]) or pos_punto[1] < 0):
#                     continue

#                 if mapa[pos_punto[0]][pos_punto[1]] != 0:
#                     continue

#                 if pos_punto in lista_cerrada:
#                     continue

#                 hijo = Punto(punto_actual, pos_punto)
#                 hijo.g = punto_actual.g + 1
#                 hijo.h = ((hijo.pos[0] - punto_final.pos[0]) ** 2) + ((hijo.pos[1] - punto_final.pos[1]) ** 2)
#                 hijo.f = hijo.g + hijo.h

#                 if any(p for p in lista_abierta if hijo.pos == p.pos and hijo.g > p.g):
#                     continue

#                 lista_abierta.append(hijo)
#         return None

#     def dev_tray(self, punto_actual):
#         tray = []
#         loc = punto_actual
#         while loc is not None:
#             tray.append(loc.pos)
#             loc = loc.padre
#         return tray[::-1]

# class Punto:
#     def __init__(self, padre=None, pos=None):
#         self.padre = padre
#         self.pos = pos
#         self.g = 0
#         self.h = 0
#         self.f = 0
#     def __eq__(self, sig):
#         return self.pos == sig.pos

# def main(args=None):
#     rclpy.init(args=args)
#     node = GridPlannerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()