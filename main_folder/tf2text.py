import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

def save_tf_static_from_bag(bag_path, output_file):
    rclpy.init()

    with rclpy.create_node('tf_static_reader') as node:
        reader = rclpy.serialization.SerializationFormatDeserializer(
            'cdr', node)
        tf_msg_type = get_message('tf2_msgs/msg/TFMessage')
        
        with rclpy.utilities.RosBagReader(bag_path) as bag_reader:
            for topic, msg, t in bag_reader.read_messages():
                if topic == '/tf_static':
                    tf_msg = deserialize_message(msg, tf_msg_type)
                    # Process the tf_static message and save to file
                    with open(output_file, 'w') as f:
                        for transform in tf_msg.transforms:
                            f.write(str(transform) + '\n')

    rclpy.shutdown()

bag_path = '/path/to/your/ros2_bag.bag'
output_file = '/path/to/output_tf_static.txt'
save_tf_static_from_bag(bag_path, output_file)
