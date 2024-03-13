import cv2
import numpy as np

class CameraCalibration:
    def __init__(self):
        # Parámetros de calibración de la cámara
        self.calibration_params = {
            "width": 640,
            "height": 360,
            # "width": 1280,
            # "height": 720,
            "k": np.array([[382.927414, 0.0, 326.270488],
                           [0.0, 383.210948, 174.61705],
                           [0.0, 0.0, 1.0]]),
            "d": np.array([-0.340658, 0.081919, 3.0e-06, 0.002878, 0.0]),
            "r": np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]]),
            "p": np.array([[256.520905, 0.0, 338.412784, 0.0],
                           [0.0, 349.866089, 173.027438, 0.0],
                           [0.0, 0.0, 1.0, 0.0]])
        }

    def undistort_image(self, input_image_path, output_image_path):
        # Cargar la imagen de entrada
        input_image = cv2.imread(input_image_path)

        # Obtener los parámetros de calibración
        width = self.calibration_params["width"]
        height = self.calibration_params["height"]
        k = self.calibration_params["k"] * 2
        d = self.calibration_params["d"]
        r = self.calibration_params["r"]
        p = self.calibration_params["p"]

        # Crear los mapas de transformación para la corrección de distorsión
        map1, map2 = cv2.initUndistortRectifyMap(
            k, d, r, p, (width, height), cv2.CV_16SC2)

        # Aplicar la corrección de distorsión a la imagen
        undistorted_image = cv2.remap(
            input_image, map1, map2, cv2.INTER_LINEAR)

        # Guardar la imagen corregida en el mismo directorio
        cv2.imwrite(output_image_path, undistorted_image)
        print(f"Imagen corregida guardada en: {output_image_path}")


def main():
    # Ruta de la imagen de entrada y salida
    input_image_path = "/workspaces/SfM/colmap_ws/rosbag_office/undistort/image_0024.jpg"
    output_image_path = "/workspaces/SfM/colmap_ws/rosbag_office/undistort/undistorted_image.jpg"

    # Crear una instancia de la clase CameraCalibration
    camera_calibration = CameraCalibration()

    # Aplicar corrección de distorsión a la imagen de entrada
    camera_calibration.undistort_image(input_image_path, output_image_path)


if __name__ == "__main__":
    main()



# frame_id: right
# height: 360
# width: 640
# distortion_model: plumb_bob
# d:
# - -0.340658
# - 0.081919
# - 3.0e-06
# - 0.002878
# - 0.0
# k:
# - 382.927414
# - 0.0
# - 326.270488
# - 0.0
# - 383.210948
# - 174.61705
# - 0.0
# - 0.0
# - 1.0
# r:
# - 1.0
# - 0.0
# - 0.0
# - 0.0
# - 1.0
# - 0.0
# - 0.0
# - 0.0
# - 1.0
# p:
# - 256.520905
# - 0.0
# - 338.412784
# - 0.0
# - 0.0
# - 349.866089
# - 173.027438
# - 0.0
# - 0.0
# - 0.0
# - 1.0
# - 0.0
# binning_x: 0
# binning_y: 0
# roi:
#   x_offset: 0
#   y_offset: 0
#   height: 0
#   width: 0
#   do_rectify: false

