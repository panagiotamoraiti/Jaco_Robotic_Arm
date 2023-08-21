import numpy as np
import pandas as pd
from PIL import Image
from matplotlib import pyplot as plt
from tensorflow.python.util import compat
from tensorflow.core.protobuf import saved_model_pb2
from google.protobuf import text_format
import pprint
import json
import sys
import os
import tensorflow as tf
import cv2
from tensorflow.keras.preprocessing.image import load_img
from tensorflow.keras.utils import img_to_array

model_path = "./../Grape Leaves disease detection/yolov4"
new_model = tf.keras.models.load_model(model_path)

# For testing the model through some sample leaf images
# directory_path = './../Grape Leaves disease detection/images/Test File'

# For real-time detection using snapshots taken from the robot camera in Unity
directory_path = './../../Jaco_arm/temp_txt'
rotate_file_path = directory_path + '/rotate.txt'
detection_file_path = directory_path + '/detection.txt'
category_file_path = directory_path + '/category.txt'
coordinates_file_path = directory_path + '/coordinates.txt'

end_effector_x = 317
end_effector_y = 354

while(True):
    for filename in os.listdir(directory_path):
        # Check if the file has an image extension
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".gif")):
            # Destroy previously created text files
            filename_no_ext, _ = os.path.splitext(filename)
            
            # Get the full path of the image file
            image_path = os.path.join(directory_path, filename)
            
            # Load image with OpenCV
            img = cv2.imread(image_path)
            
            if img is not None:
                h, w = img.shape[:2]

                height = 256
                width = 256

                # Load image in PIL image form for the model
                image = load_img(image_path, target_size=(height, width))
                image = img_to_array(image)/255.0
                image = np.expand_dims(image, axis=0)
                
                
                # Back to detecting
                prediction = new_model.predict(image)
                detection = False
                
                # Detect markers
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
                parameters = cv2.aruco.DetectorParameters()
                detector = cv2.aruco.ArucoDetector(dictionary, parameters)
                markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img)
                cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)
                # print(markerCorners)
                
                marker_width = 75.0    # default
                for corners in markerCorners:
                    # Compute the Euclidean distance between corners to find width and height
                    marker_width = abs(corners[0][0][0] - corners[0][1][0])
                    marker_height = abs(corners[0][0][1] - corners[0][3][1])
                
                transform_ratio = 0.1/marker_width

                if len(prediction[0]) > 0:
                    detection = True
                    txt_list = []
                    for i in range(len(prediction[0])):
                        if i == 0:
                            if (os.path.isfile(coordinates_file_path)):
                                os.remove(coordinates_file_path)
                            
                         #print(prediction[0][i])
                        (sy, sx, ey, ex, conf) = prediction[0][i]
                        #print(sx, sy, ex, ey, conf)

                        sx = int(sx * w)
                        sy = int(sy * h)
                        ex = int(ex * w)
                        ey = int(ey * h)
                        
                        box_w = ex - sx
                        box_h = ey - sy
                        
                        # Find bigger side, and rotate gripper if needed
                        if box_w > box_h+5:
                             with open(rotate_file_path, 'w') as file:
                                 file.write(f"True")
                        else:
                            with open(rotate_file_path, 'w') as file:
                                 file.write(f"False")
                            
                            
                        center_x = (sx + ex) // 2
                        center_y = (sy + ey) // 2
                        
                        # Y axis in Opencv is -Z axis in Unity 
                        move_x = (end_effector_x - center_x) * transform_ratio
                        move_z = -(end_effector_y - center_y) * transform_ratio
                        
                        move_x = round(move_x, 3)
                        move_z = round(move_z, 3)

                        # Draw a small circle at the center of the rectangle (only for the 1st prediction)
                        if i == 0:
                            circle_radius = 3
                            circle_color = (255, 0, 0)  
                            cv2.circle(img, (center_x, center_y), circle_radius, circle_color, -1)

                        
                        with open(coordinates_file_path, 'a+') as output_file:
                            if (center_x, center_y) not in txt_list:
                                output_file.write(f"{move_x}\n{move_z}\n")
                                txt_list.append((center_x, center_y))

                        cv2.rectangle(img, (sx, sy), (ex,ey), (0, 255, 0), 2)
                else:
                    if (os.path.isfile(coordinates_file_path)):
                        os.remove(coordinates_file_path)
                    
                    with open(coordinates_file_path, 'a+') as output_file:
                        output_file.write(f"0\n0\n")
                        
                with open(category_file_path, 'w') as output_file:
                    output_file.write(f"other")

                with open(detection_file_path, 'w') as output_file:
                    output_file.write(f"{detection}")
                      
                cv2.imwrite('./../output_image.jpg', img)
                # Remove image
                os.remove(image_path)

