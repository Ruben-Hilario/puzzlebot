#!/usr/bin/env python3
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

#from puzzlebot_control.

# - Lectura de la señal de audio (16KHz)
# - Filtro de preenfasis
# - Ventana de Hamming 
# - Inicio y final de cada palabra
# - 

class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__('void_cmd_node')
        media_path = os.path.join(get_package_share_directory('puzzlebot_control'), 'media')





def main(args=None):
    rclpy.init(args=args)
    voice_cmd_node = VoiceCmdNode()
    rclpy.spin(voice_cmd_node)
    voice_cmd_node.destroy_node()
    rclpy.shutdown()

if __name == '__main__':
    main()
