import numpy as np
import pandas as pd
import tensorflow as tf
from PIL import Image
from matplotlib import pyplot as plt
from tensorflow.python.util import compat
from tensorflow.core.protobuf import saved_model_pb2
from google.protobuf import text_format
import pprint
import json
import sys
import os
import cv2

from object_detection.utils import visualization_utils as vis_util
from object_detection.utils import dataset_util, label_map_util
from object_detection.protos import string_int_label_map_pb2

# reconstruct frozen graph
def reconstruct(pb_path):
    if not os.path.isfile(pb_path):
        print("Error: %s not found" % pb_path)

    print("Reconstructing Tensorflow model")
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(pb_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    print("Success!")
    return detection_graph

# visualize detection
def image2np(image):
    (w, h) = image.size
    return np.array(image.getdata()).reshape((h, w, 3)).astype(np.uint8)

def image2tensor(image):
    npim = image2np(image)
    return np.expand_dims(npim, axis=0)

def detect(detection_graph, test_image_path, img):
  with detection_graph.as_default():
    gpu_options = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=0.01)
    with tf.compat.v1.Session(graph=detection_graph,config=tf.compat.v1.ConfigProto(gpu_options=gpu_options)) as sess:
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        if not img:
          image = Image.open(test_image_path)
        else:
          image = test_image_path
          
        flag = False

        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image2tensor(image)}
        )

        w, h = image.size
 
        first_box_coordinates = boxes[0][0]
        y_min, x_min, y_max, x_max = first_box_coordinates
 
        x_min = int(x_min * w)
        y_min = int(y_min * h)
        x_max = int(x_max * w)
        y_max = int(y_max * h)
 
        # Calculate the center coordinates
        center_y = (y_min + y_max) // 2
        center_x = (x_min + x_max) // 2        
        
        npim = image2np(image)
        
        vis_util.visualize_boxes_and_labels_on_image_array(
            npim,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=5,
            min_score_thresh=.3)
            
        # Draw a circle at the center of the bounding box
        center = (center_x * width, center_y * height)
        circle_color = (255, 0, 0)  
        circle_radius = 3 
        circle = cv2.circle(npim, (center_x, center_y), circle_radius, circle_color, -1)
        
        with open(output_file_path, 'a+') as output_file:
                check = (center_x, center_y)
                for line in output_file:
                    if line.strip() == check.strip():
                        flag = True
                if flag == False:
                    output_file.write(f"({center_x}, {center_y})\n")
        
        cv2.imshow("image", npim)
        cv2.waitKey(0)
        
        cv2.destroyAllWindows()


DATA_DIR = './../Garbage Detection'
ANNOTATIONS_FILE = os.path.join(DATA_DIR, 'annotations.json')
NCLASSES = 60

with open(ANNOTATIONS_FILE) as json_file:
    data = json.load(json_file)

categories = data['categories']

#print('Building label map from examples')

labelmap = string_int_label_map_pb2.StringIntLabelMap()
for idx,category in enumerate(categories):
    item = labelmap.item.add()
    # label map id 0 is reserved for the background label
    item.id = int(category['id'])+1
    item.name = category['name']

with open('./labelmap.pbtxt', 'w') as f:
    f.write(text_format.MessageToString(labelmap))

# print('Label map witten to labelmap.pbtxt')

# with open('./labelmap.pbtxt') as f:
#    pprint.pprint(f.readlines())

label_map = label_map_util.load_labelmap('labelmap.pbtxt')
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NCLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

detection_graph = reconstruct("./../Garbage Detection/ssd_mobilenet_v2_taco_2018_03_29.pb")

#directory_Path = './../Garbage Detection/images/batch_1'
# For real-time detection using snapshots taken from the robot camera in Unity
directory_Path = './../../Jaco_arm/Screenshots'

while(True):
    for filename in os.listdir(directory_Path):
        # Check if the file has an image extension
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".gif")):
            # Destroy previously created text files
            filename_no_ext, _ = os.path.splitext(filename)
            output_file_path = directory_Path + '/' + filename_no_ext + ".txt"
            
            if (os.path.isfile(output_file_path)):
                os.remove(output_file_path)
                
            # Get the full path of the image file
            image_path = os.path.join(directory_Path, filename)

            image = cv2.imread(image_path)
            height, width, _ = image.shape
            resized_image = cv2.resize(image, (height//4, width//4))
            height, width, _ = resized_image.shape
            #cv2.imwrite('./../Garbage Detection/images/output_image.jpg', resized_image)
            cv2.imwrite(directory_Path + '/' + filename + '_resized.jpg', resized_image)

            detect(detection_graph, directory_Path + '/' + filename + '_resized.jpg', False)

       

