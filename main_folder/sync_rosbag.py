import os
import typing as tp
import cv2
import numpy as np
import message_filters
import rosbag2_py
import sensor_msgs
import tf2_msgs
import typer
import piexif
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tqdm import tqdm
from sensor_msgs.msg import CameraInfo

import zstandard


def decompress_file(input_path: str, output_path: str):
    """! Decompress a file using zstandard
    @param input_path (str) path to the compressed file
    @param output_path (str) path to the decompressed file
    """
    dctx = zstandard.ZstdDecompressor()

    with open(input_path, "rb") as ifh, open(output_path, "wb") as ofh:
        dctx.copy_stream(ifh, ofh)


# Avoid printing matrices with scientific notation
np.set_printoptions(suppress=True)


def get_rosbag_options(path: str, serialization_format="cdr"):
    """! Ros2 bag options
    @param path (str) path to rosbag2
    @param serialization_format (str, optional) format . Defaults to "cdr".
    @return tuple of storage and converter options
    """
    extension_file = path.split(".")[-1]
    if extension_file == "mcap":
        storage_id = "mcap"
    elif extension_file == "db3":
        storage_id = "sqlite3"
    else:
        raise ValueError(f"Unknown extension file {extension_file}")
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )
    return (storage_options, converter_options)


class RosBagSerializer(object):
    def __init__(
        self,
        rosbag_path: str,
        topics: tp.List[str],
        queue_size: int = 30,
        time_delta: float = 0.1,
        imshow: bool = True,
        undistort: bool = True,
        fps: int = 10,
        verbose=True,
    ):
        """! Serialize a rosbag into different synchronized videos per topic.
        Optionally, it can also save a video with all the topics concatenated.

        @param rosbag_path (str) path to the rosbag to serialize.
        @param topics (list) list of topics to serialize and synchronize.
        @param queue_size (int, optional) queue size for the message filters. Defaults to 30.
        @param time_delta (float, optional) time delta for the message filters. Defaults to 0.1.
        @param imshow (bool, optional) show the images while serializing. Defaults to False.
        @param undistort (bool, optional) undistort the images using the camera info. Defaults to True.
        @param concat_images (bool, optional) concatenate all the images in a single video. Defaults to False.
        @param fps (int, optional) fps for the videos. Defaults to 10.
        @param verbose (bool, optional) print information while serializing. Defaults to True.
        """

        self.rosbag_path = rosbag_path
        self.topics = topics
        self.queue_size = queue_size
        self.time_delta = time_delta
        self.imshow = imshow
        self.fps = fps
        self.undistort = undistort
        self.verbose = verbose

        self.rosbag_dir = os.path.dirname(rosbag_path)
        self.bridge = CvBridge()

        self.output_dir = os.path.join(os.path.expanduser('/workspaces/SfM/colmap_ws/rosbag_office'), 'images')
        os.makedirs(self.output_dir, exist_ok=True)

        self.imu_gps_output_dir = os.path.join(os.path.expanduser('/workspaces/SfM/colmap_ws'), 'rosbag_office')
        os.makedirs(self.imu_gps_output_dir, exist_ok=True)

        self.cams_params = {}  # for all usb cameras

        # Check if rosbag is compressed
        if rosbag_path.endswith(".zstd"):
            decompressed_rosbag_path = rosbag_path.replace(".zstd", "")
            decompress_file(rosbag_path, decompressed_rosbag_path)
            self.rosbag_path = decompressed_rosbag_path

        storage_options, converter_options = get_rosbag_options(self.rosbag_path)

        self.rosbag = rosbag2_py.SequentialReader()
        self.rosbag.open(storage_options, converter_options)
        topic_types = self.rosbag.get_all_topics_and_types()
        # Create a map for quicker lookup
        self.topic_types_map = {
            topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
        }
        for topic, msg_type in self.topic_types_map.items():
            print(f"Topic: {topic}, Type: {msg_type}")
        # print(self.topic_types_map)

        if not set(self.topics).issubset(set(self.topic_types_map.keys())):
            raise ValueError(
                "The topics you provided are not in the bag file. "
                "Please check the topics you provided and the bag file. "
                f"Topics in bag: {list(self.topic_types_map.keys())}"
            )

        # Idea taken from C++ example http://wiki.ros.org/rosbag/Cookbook#Analyzing_Stereo_Camera_Data
        self.filters_dict = {topic: message_filters.SimpleFilter() for topic in topics}

        self.ts = message_filters.ApproximateTimeSynchronizer(
            list(self.filters_dict.values()), queue_size, time_delta
        )

        self.ts.registerCallback(self.sync_callback)

        # tqdm progress bar
        if self.verbose:
            self.pbar = tqdm(desc="Serializing rosbag data... ")

        self.video_writers = {}
        self.i = 0  # Inicializar el contador


    def sync_callback(self, *msgs):
        """! Callback for the approximate time synchronizer of msgs
        @param msgs (list of msgs) list of msgs from the topics
        """
        if self.verbose:
            self.pbar.update(1)

        img_data = {}
        imu_gps_data = {}

        # Iterate over arguments, each argument is a different msg
        for topic, msg in zip(self.topics, msgs):
            # Parse msg depending on its type
            msg_info_dict = self.parse_msg(msg, topic)
            
            # Get the timestamp of the message
            timestamp = msg.header.stamp if hasattr(msg.header, 'stamp') else None

            # Print the topic, timestamp, and any other relevant information
            #print(f"Topic: {topic}, Timestamp: {timestamp}")

            if isinstance(msg, sensor_msgs.msg.CompressedImage) or isinstance(msg, sensor_msgs.msg.Image):
                #img_data[topic] = msg_info_dict.get("data")
                img_data[topic] = msg_info_dict.pop("data")
                cv2.imshow(topic, img_data[topic])
                cv2.waitKey(1)

                # # Guardar las imágenes descomprimidas en el directorio de salida
                # for topic, img_data in img_data.items():
                #     #image_filename = f"{topic}_{msg.header.stamp}.jpg"
                #     #image_filename = f"{msg.header.stamp}.jpg"
                #     image_filename = f"image_{self.i+1:04d}.jpg"
                #     image_path = os.path.join(self.output_dir, image_filename)
                #     cv2.imwrite(image_path, img_data) 
            # elif isinstance(msg, sensor_msgs.msg.Imu):
            #     imu_data = msg_info_dict.get("data")
            #     if imu_data:
            #         imu_file_path = os.path.join(self.imu_gps_output_dir, "imu_data.txt")
            #         with open(imu_file_path, "a") as imu_file:
            #             # imu_file.write(f"Timestamp: {timestamp}, IMU Data: {imu_data}\n")
            #             imu_file.write(f"{image_filename}, IMU Data: {imu_data}\n")
            elif isinstance(msg, sensor_msgs.msg.NavSatFix):
                gps_data = msg_info_dict.get("data")
                if gps_data:
                    imu_gps_data['gps'] = gps_data
                # if gps_data:
                    # gps_file_path = os.path.join(self.imu_gps_output_dir, "gps_data.txt")
                    # with open(gps_file_path, "a") as gps_file:
                    #     gps_file.write(f"{image_filename}, GPS Data: {gps_data}\n")

        # Guardar las imágenes en el directorio de salida con intrinsics y GPS
        for topic, img_data in img_data.items():
            image_filename = f"image_{self.i+1:04d}.jpg"
            image_path = os.path.join(self.output_dir, image_filename)
            cv2.imwrite(image_path, img_data)

            exif_dict = piexif.load(image_path)
            # Agregar información sobre la distancia focal y los puntos centrales
            fx = self.cams_params[topic]["k"][0, 0]
            fy = self.cams_params[topic]["k"][1, 1]
            cx = self.cams_params[topic]["k"][0, 2]
            cy = self.cams_params[topic]["k"][1, 2]
            exif_dict["Exif"][piexif.ExifIFD.FocalLength] = (int(fx*1000), 1000)  # Distancia focal en milímetros
            #exif_dict["Exif"][piexif.ExifIFD.PixelXDimension] = int(cx)  # Punto central en el eje x
            #exif_dict["Exif"][piexif.ExifIFD.PixelYDimension] = int(cy)  # Punto central en el eje y


            # Guardar la información de GPS en el EXIF de la imagen
            if 'gps' in imu_gps_data:
                #exif_dict = piexif.load(image_path)
                latitude = imu_gps_data['gps']['latitude']
                longitude = imu_gps_data['gps']['longitude']
                altitude = imu_gps_data['gps']['altitude']
                lat = abs(latitude)
                lon = abs(longitude)
                lat_deg = int(lat)
                lat_min = int((lat - lat_deg) * 60)
                lat_sec = int(((lat - lat_deg) * 60 - lat_min) * 60 * 1000)
                lon_deg = int(lon)
                lon_min = int((lon - lon_deg) * 60)
                lon_sec = int(((lon - lon_deg) * 60 - lon_min) * 60 * 1000)
                # Asignar valores de latitud, longitud y altitud al diccionario EXIF
                exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] = 'N' if latitude >= 0 else 'S'
                exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = ((lat_deg, 1), (lat_min, 1), (lat_sec, 1000))
                exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] = 'E' if longitude >= 0 else 'W'
                exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = ((lon_deg, 1), (lon_min, 1), (lon_sec, 1000))
                exif_dict["GPS"][piexif.GPSIFD.GPSAltitude] = (int(altitude*1000), 1000)  # Altitud en metros
                

            print(exif_dict)
            # Save updated EXIF data back to the image
            exif_bytes = piexif.dump(exif_dict)
            piexif.insert(exif_bytes, image_path)
            imu_gps_data.pop("gps")
                
        self.i += 1

    def parse_msg(self, msg: tp.Any, topic: str) -> tp.Dict[str, tp.Any]:
        """
        Parses msg depending on its type.
        @param msg (any) message to be parsed
        @param topic (str) topic of the message
        @return (dict) message as dictionary
        """
        if isinstance(msg, sensor_msgs.msg.Image):
            return self.parse_image(msg, topic)
        elif isinstance(msg, sensor_msgs.msg.CompressedImage):
            return self.parse_compressed_image(msg, topic)
        elif isinstance(msg, sensor_msgs.msg.CameraInfo):
            return self.parse_camera_info(msg)
        elif isinstance(msg, sensor_msgs.msg.Imu):
            return self.parse_imu_info(msg)
        elif isinstance(msg, sensor_msgs.msg.NavSatFix):
            return self.parse_gps_info(msg)
        # elif isinstance(msg, tf2_msgs.msg.TFMessage):
        #     return self.parse_tf_info(msg)
        else:
            raise ValueError(f"Unsupported message type {type(msg)}")
        
    
    def parse_camera_info(
        self, msg: sensor_msgs.msg.CameraInfo
    ) -> tp.Dict[str, tp.Any]:
        """Parses camera info msg and saves it in self.stereo_cam_model
        @param msg (CameraInfo) camera info message
        @param topic (str) topic of the message
        @return (dict) empty dict
        """
        params = {}

        params["dim"] = (int(msg.width), int(msg.height))
        params["k"] = np.array(msg.k).reshape((3, 3))
        params["p"] = np.array(msg.p).reshape((3, 4))
        params["distortion_model"] = msg.distortion_model
        params["d"] = np.array(msg.d)
        params["r"] = np.array(msg.r).reshape((3, 3))

        if params["distortion_model"] in ("equidistant", "fisheye"):
            initUndistortRectifyMap_fun = cv2.fisheye.initUndistortRectifyMap
        else:
            initUndistortRectifyMap_fun = cv2.initUndistortRectifyMap
        params["map1"], params["map2"] = initUndistortRectifyMap_fun(
            params["k"],
            params["d"],
            np.eye(3),
            params["k"],
            params["dim"],
            cv2.CV_16SC2,
        )

        return params
    
    def scale_calibration(self, params: dict, factor: int) -> dict:
        """! Scale calibration parameters

        @param params (dict) calibration parameters
        @param factor (int) scale factor

        @return scaled calibration parameters
        """
        if factor != 1:
            params["k"] = params["k"] * factor
            params["k"][2, 2] = 1
        if params["distortion_model"] in ("equidistant", "fisheye"):
            initUndistortRectifyMap_fun = cv2.fisheye.initUndistortRectifyMap
        else:
            initUndistortRectifyMap_fun = cv2.initUndistortRectifyMap
        params["map1"], params["map2"] = initUndistortRectifyMap_fun(
            params["k"] * factor,
            params["d"],
            params["r"],
            params["k"] * factor,
            params["dim"],
            # params["dim"] * factor,
            cv2.CV_16SC2,
        )
        return params

    def parse_image(
        self, msg: sensor_msgs.msg.Image, topic: str
    ) -> tp.Dict[str, tp.Any]:
        """
        Parses image message
        @param msg (Image) image message
        @param topic (str) topic of the message
        @return (dict) image message as dictionary with the path to the image
            and the image itself as numpy array
        """
        # Convert image to cv2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # only undistort usb camera images, stereo is already undistorted
        if self.cams_params.get(topic) and self.undistort:
            print("distorted")
            cv_image = cv2.remap(
                cv_image,
                self.cams_params[topic]["map1"],
                self.cams_params[topic]["map2"],
                cv2.INTER_LINEAR,
            )
        return {"data": cv_image}
    
    def parse_compressed_image(
        self, msg: sensor_msgs.msg.CompressedImage, topic: str
    ) -> tp.Dict[str, tp.Any]:
        np_arr = np.array(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # only undistort usb camera images, stereo is already undistorted
        if self.cams_params.get(topic) and self.undistort:
            cv_image = cv2.remap(
                cv_image,
                self.cams_params[topic]["map1"],
                self.cams_params[topic]["map2"],
                cv2.INTER_LINEAR,
            )
        return {"data": cv_image}

    def parse_imu_info(self, msg: sensor_msgs.msg.Imu) -> tp.Dict[str, tp.Any]:
        """Parses IMU message."""
        imu_data = {
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            }
        }
        return {"data": imu_data}
    
    def parse_gps_info(self, msg: sensor_msgs.msg.NavSatFix) -> tp.Dict[str, tp.Any]:
        """Parses GPS message."""
        gps_data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude
        }
        return {"data": gps_data}
    
    # def parse_tf_info(self, msg: tf2_msgs.msg.TFMessage) -> tp.Dict[str, tp.Any]:
    #     """Parses TF message."""
    #     tf_data = {
    #         "child": msg.child_frame_id,
    #         "translation": {
    #             "x": msg.translation.x,
    #             "y": msg.translation.y,
    #             "z": msg.translation.z,
    #         },
    #         "rotation": {
    #             "x": msg.rotation.x,
    #             "y": msg.rotation.y,
    #             "z": msg.rotation.z,
    #             "w": msg.rotation.w,
    #         }
    #     }
    #     return {"data": tf_data}

    
    def process_rosbag(self):
        """Processes the rosbag file, it starts reading message by
        message and sending them to the approximate synchronizer that
        will trigger the synchronization of the messages function.
        """

        camera_info_topics = [topic for topic in self.topics if "image_raw" in topic]
        camera_info_topics = [topic.replace("image_raw", "camera_info") for topic in camera_info_topics]

        # Diccionario para rastrear si se ha encontrado al menos un mensaje de cada topic de camera_info_topics
        found_data_for_topics = {topic: False for topic in camera_info_topics}

        # Read rosbag until all camera info needed are found
        while self.rosbag.has_next() and not all(found_data_for_topics.values()):
            (topic, data, t) = self.rosbag.read_next()
            print(topic)
            
            if topic not in camera_info_topics:
                continue

            msg_type = get_message(self.topic_types_map[topic])
            msg = deserialize_message(data, msg_type)
            #print(topic)

            corresponding_topic = topic.replace("camera_info", "image_raw")
            self.cams_params[corresponding_topic] = self.parse_camera_info(msg)

            # Marcar el topic como encontrado
            found_data_for_topics[topic] = True

        print(self.cams_params)
        
        # Continue reading the rosbag
        # We will lost some frames but this way all will have instrinsincs
        # Another way to do this keeping all frames is by creating a new SequentialReader object

        while self.rosbag.has_next():
            (topic, data, t) = self.rosbag.read_next()
            
            #this is to avoid reading customized messages that can show errors
            if topic not in self.topics:
                continue

            print(topic)
            msg_type = get_message(self.topic_types_map[topic])
            msg = deserialize_message(data, msg_type)

            # # Camerainfo topics are too slow to sync with other topics
            # if isinstance(msg, CameraInfo):
            #     #print(topic)
            #     corresponding_topic = topic.replace("camera_info", "image_raw")
            #     #print(corresponding_topic)
            #     self.cams_params[corresponding_topic] = self.parse_camera_info(msg)
            #     #print(self.cams_params)
            #     #if corresponding_topic == "/video_mapping/right/image_raw":
            #     #    self.cams_params[corresponding_topic] = self.scale_calibration(self.cams_params[corresponding_topic], 2)

            if topic in self.filters_dict:
                filter_obj = self.filters_dict[topic]
                filter_obj.signalMessage(msg)

            

            # DEBUG
            # if isinstance(msg, sensor_msgs.msg.Image):
            #     cv_img = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            #     cv2.imshow(f"{topic}-debug", cv_img)
            #     cv2.waitKey(1)


def main(
    sync_topics: tp.List[str] = [
        # include the image_raw and the camera_info of the cameras you want to synchronize
        # include /fix if you want GPS data
        "/camera/color/image_raw",
        #"/camera/color/camera_info",
        #"/video_mapping/left/image_raw",
        #"/video_mapping/left/camera_info",
        #"/video_mapping/right/image_raw",
        #"/video_mapping/right/camera_info",
        #"/video_mapping/rear/image_raw",
        #"/video_mapping/rear/camera_info",
        #"/imu/data",
        "/fix",
        #"/tf_static"
    ],
    fps: int = 10,
    undistort: bool = False,
    debug: bool = False,
    imshow: bool = False,
):
    """
    Main function for processing the rosbag files.
    @sync_topics: list of topics to process
    @bag_path: list of paths to the rosbag files to process
    @fps: fps of the output videos
    @undistort: if True, undistorts the images
    @concat_images: if True, concatenates the images horizontally
    @debug: if True, waits for debugger to attach
    @imshow: if True, shows the images
    """

    bag_path = os.path.abspath(os.path.expanduser("/workspaces/SfM/colmap_ws/rosbag_office/rosbag/sfm_0.mcap"))
    
    if debug:
        import debugpy  # pylint: disable=import-error

        print("Waiting for debugger...")
        debugpy.listen(5678)
        debugpy.wait_for_client()

    rosbag_serializer = RosBagSerializer(
        bag_path,
        sync_topics,
        time_delta=0.1,
        fps=fps,
        undistort=undistort,
        imshow=imshow,
    )
    rosbag_serializer.process_rosbag()


if __name__ == "__main__":
    typer.run(main)
