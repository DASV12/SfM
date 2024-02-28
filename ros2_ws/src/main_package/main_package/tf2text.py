import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import pycdr

def save_tf_static_from_bag(bag_path, output_file):
    # Inicializar el nodo de ROS
    rclpy.init()
    node = rclpy.create_node('tf_static_reader')

    # Abrir el archivo de salida en modo escritura
    with open(output_file, 'w') as f:
        # Abrir el archivo ROS 2 Bag
        with pycdr.open_bag(bag_path) as bag:
            # Iterar sobre los mensajes del archivo ROS 2 Bag
            for topic, _, data in bag.read_messages():
                # Verificar si es un mensaje de tipo TFMessage
                if topic == '/tf_static':
                    # Obtener el mensaje TFMessage
                    msg_type = get_message('tf2_msgs/msg/TFMessage')
                    tf_message = msg_type()
                    tf_message.deserialize(data)
                    # Escribir las transformaciones en el archivo de salida
                    for transform in tf_message.transforms:
                        f.write(f'{transform}\n')

    # Cerrar el nodo de ROS
    node.destroy_node()
    rclpy.shutdown()


# Ruta del archivo ROS 2 Bag
bag_path = '/serial/main_folder/bag.mcap'
# Ruta del archivo de salida de texto
output_file = '/serial/colmap_ws/rosbag_video/tf_data.txt'

# Llamar a la función para guardar tf_static del Bag en el archivo de texto
save_tf_static_from_bag(bag_path, output_file)
