import os
import re
import piexif

def add_gps_info_to_image(image_path, latitude, longitude, altitude):
    exif_dict = piexif.load(image_path)

    # Convert latitude and longitude to GPS coordinates format
    lat = abs(latitude)
    lon = abs(longitude)
    lat_deg = int(lat)
    lat_min = int((lat - lat_deg) * 60)
    lat_sec = int(((lat - lat_deg) * 60 - lat_min) * 60 * 1000)
    lon_deg = int(lon)
    lon_min = int((lon - lon_deg) * 60)
    lon_sec = int(((lon - lon_deg) * 60 - lon_min) * 60 * 1000)

    # Set GPS data
    exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] = 'N' if latitude >= 0 else 'S'
    exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = ((lat_deg, 1), (lat_min, 1), (lat_sec, 1000))
    exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] = 'E' if longitude >= 0 else 'W'
    exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = ((lon_deg, 1), (lon_min, 1), (lon_sec, 1000))

    # Save updated EXIF data back to the image
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, image_path)

# Directorio donde se encuentran las imágenes
image_directory = "/serial/colmap_ws/rosbag_video/images"

# Directorio donde se encuentra el archivo de texto con los datos de GPS
gps_file_path = "/serial/colmap_ws/rosbag_video/gps_data.txt"

# Expresión regular para buscar datos de GPS en el archivo de texto
gps_data_regex = re.compile(r"image_\d+\.jpg, GPS Data: {'latitude': ([0-9.-]+), 'longitude': ([0-9.-]+), 'altitude': ([0-9.-]+)}")

# Leer el archivo de texto y procesar los datos de GPS
with open(gps_file_path, "r") as file:
    for line in file:
        match = gps_data_regex.search(line)
        if match:
            # Obtener los valores de GPS
            latitude = float(match.group(1))
            longitude = float(match.group(2))
            altitude = float(match.group(3))
            
            # Construir la ruta de la imagen correspondiente
            image_filename = line.split(",")[0]
            image_path = os.path.join(image_directory, image_filename)

            # Verificar si la imagen existe y agregar información de GPS al EXIF
            if os.path.exists(image_path):
                add_gps_info_to_image(image_path, latitude, longitude, altitude)
                print(f"Se agregó información de GPS a {image_filename}")
            else:
                print(f"No se encontró la imagen {image_filename}")
