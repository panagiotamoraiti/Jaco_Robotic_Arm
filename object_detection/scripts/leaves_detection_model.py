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
# directory_Path = './../Grape Leaves disease detection/images/Test File'

# For real-time detection using snapshots taken from the robot camera in Unity
directory_Path = './../../Jaco_arm/Screenshots'

end_effector_x = 532
end_effector_y = 348
transform_ratio = 0.15/100

while(True):
    for filename in os.listdir(directory_Path):
        # Check if the file has an image extension
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".gif")):
            # Destroy previously created text files
            filename_no_ext, _ = os.path.splitext(filename)
            output_file_path = directory_Path + '/' + filename_no_ext + ".txt"
            
            '''if (os.path.isfile(output_file_path)):
                os.remove(output_file_path)
                
            with open(output_file_path, 'w'):
                pass'''
            
            # Get the full path of the image file
            image_path = os.path.join(directory_Path, filename)

            height = 256
            width = 256

            image = load_img(image_path, target_size=(height, width))
            image = img_to_array(image)/255.0
            image = np.expand_dims(image, axis=0)

            prediction = new_model.predict(image)
            #print(prediction)

            img = cv2.imread(image_path)
            h, w = img.shape[:2]

            txt_list = []
            for i in range(len(prediction[0])):
                if i == 0:
                    if (os.path.isfile(output_file_path)):
                        os.remove(output_file_path)
                    
                 #print(prediction[0][i])
                (sy, sx, ey, ex, conf) = prediction[0][i]
                #print(sx, sy, ex, ey, conf)

                sx = int(sx * w)
                sy = int(sy * h)
                ex = int(ex * w)
                ey = int(ey * h)

                center_x = (sx + ex) // 2
                center_y = (sy + ey) // 2
                
                # Y axis in Opencv is -Z axis in Unity 
                move_x = (end_effector_x - center_x) * transform_ratio
                move_z = -(end_effector_y - center_y) * transform_ratio
                
                move_x = round(move_x, 3)
                move_z = round(move_z, 3)

                # Draw a small circle at the center of the rectangle
                circle_radius = 3
                circle_color = (255, 0, 0)  
                cv2.circle(img, (center_x, center_y), circle_radius, circle_color, -1)

                
                with open(output_file_path, 'a+') as output_file:
                    if (center_x, center_y) not in txt_list:
                        # output_file.write(f"({center_x}, {center_y})\n")
                        output_file.write(f"{move_x}\n{move_z}\n")
                        txt_list.append((center_x, center_y))

                cv2.rectangle(img, (sx, sy), (ex,ey), (0, 255, 0), 2)

            cv2.imshow("image", img)
            cv2.waitKey(100)

