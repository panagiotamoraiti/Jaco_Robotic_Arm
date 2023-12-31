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

end_effector_x = 317
end_effector_y = 354

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
  detection = False
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

        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image2tensor(image)}
        )

        w, h = image.size
        
        npim = image2np(image)
        npim = cv2.cvtColor(npim, cv2.COLOR_BGR2RGB)
        
        if (num>0):
            first_box_coordinates = boxes[0][0]
            y_min, x_min, y_max, x_max = first_box_coordinates

            x_min = int(x_min * w)
            y_min = int(y_min * h)
            x_max = int(x_max * w)
            y_max = int(y_max * h)
            
            box_w = x_max - x_min
            box_h = y_max - y_min
            
            # Find bigger side, and rotate gripper if needed
            if box_w > box_h+5:
                 with open(rotate_file_path, 'w') as file:
                     file.write(f"True")
            else:
                with open(rotate_file_path, 'w') as file:
                     file.write(f"False")

            # Calculate the center coordinates
            center_y = (y_min + y_max) // 2
            center_x = (x_min + x_max) // 2   
            
            # Detect markers
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(npim)
            cv2.aruco.drawDetectedMarkers(npim, markerCorners, markerIds)
            # print(markerCorners)
            
            marker_width = 75.0    # default

            for corners in markerCorners:
                # Compute the Euclidean distance between corners to find width and height
                marker_width = abs(corners[0][0][0] - corners[0][1][0])
                marker_height = abs(corners[0][0][1] - corners[0][3][1])
            
            transform_ratio = 0.1/marker_width     
		            
            # Y axis in Opencv is -Z axis in Unity 
            move_x = (end_effector_x - center_x) * transform_ratio
            move_z = -(end_effector_y - center_y) * transform_ratio

            move_x = round(move_x, 3)
            move_z = round(move_z, 3)

            vis_util.visualize_boxes_and_labels_on_image_array(
	            npim,
	            np.squeeze(boxes),
	            np.squeeze(classes).astype(np.int32),
	            np.squeeze(scores),
	            category_index,
	            use_normalized_coordinates=True,
	            line_thickness=5,
	            min_score_thresh=.3)
	            
            if scores[0][0] >= 0.3:
                detection = True
                # Draw a circle at the center of the bounding box
                center = (center_x * width, center_y * height)
                circle_color = (255, 0, 0)  
                circle_radius = 3 
                circle = cv2.circle(npim, (center_x, center_y), circle_radius, circle_color, -1)

                if (os.path.isfile(coordinates_file_path)):
                    os.remove(coordinates_file_path)
	                
                with open(coordinates_file_path, 'a+') as output_file:
                    output_file.write(f"{move_x}\n{move_z}\n")

        else:
            if (os.path.isfile(coordinates_file_path)):
                os.remove(coordinates_file_path)
                
            with open(coordinates_file_path, 'a+') as output_file:
                output_file.write(f"0\n0\n")
        
        #cv2.imshow("image", npim)
        cv2.imwrite('./../output_image.jpg', npim)
        #cv2.waitKey(1) 
        #cv2.destroyAllWindows()

        # Write the correct category in a .txt
        first_detected_class = int(classes[0][0])
        label = category_index[first_detected_class]['name']
        # print("Detected Label:", label)

        # Search for the label in the categorized lists
        found_in_category = None
        for category, item_names in [
            ('plastic', plastic_item_names),
            ('glass', glass_item_names),
            ('metal', metal_item_names),
            ('paper', paper_item_names),
            ('other', other_item_names)
        ]:
            if label in item_names:
                found_in_category = category
                break

        #print(f"The label '{label}' was found in the '{found_in_category.capitalize()}' category.")
        with open(category_file_path, 'w') as output_file:
            output_file.write(f"{found_in_category}")

        with open(detection_file_path, 'w') as output_file:
            output_file.write(f"{detection}")

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

# Make list with 5 categories
# Define categories
categories = {
    'plastic': ['Plastic', 'Squeezable tube', 'Tupperware', 'Spread tub', 'Crisp packet', 'Garbage bag', 'Single-use carrier bag', 'Six pack rings'],
    'glass': ['Glass'],
    'metal': ['Metal', 'can', 'Aluminium ', 'Pop tab', 'Aerosol'],
    'paper': ['Paper', 'box', 'container', 'carton', 'Tissues', 'Carded blister pack', 'Foam cup', 'Toilet tube'],
}

# Categorize the class names
categorized_categories = {category: [] for category in categories}
categorized_categories['other'] = []

for class_id, class_info in category_index.items():
    class_name = class_info['name']
    for category, keywords in categories.items():
        if any(keyword.lower() in class_name.lower() for keyword in keywords):
            categorized_categories[category].append(class_info)
            break
    else:
        categorized_categories['other'].append(class_info)

# Separate lists for each category
plastic_list = categorized_categories['plastic']
glass_list = categorized_categories['glass']
metal_list = categorized_categories['metal']
paper_list = categorized_categories['paper']
other_list = categorized_categories['other']

# Extract item names
plastic_item_names = [item['name'] for item in plastic_list]
glass_item_names = [item['name'] for item in glass_list]
metal_item_names = [item['name'] for item in metal_list]
paper_item_names = [item['name'] for item in paper_list]
other_item_names = [item['name'] for item in other_list]

detection_graph = reconstruct("./../Garbage Detection/ssd_mobilenet_v2_taco_2018_03_29.pb")

#directory_path = './../Garbage Detection/images/batch_1'
# For real-time detection using snapshots taken from the robot camera in Unity
directory_path = './../../Jaco_arm/temp_txt'
rotate_file_path =  directory_path + '/rotate.txt'
category_file_path = directory_path + '/category.txt'
detection_file_path = directory_path + '/detection.txt'
coordinates_file_path = directory_path + '/coordinates.txt'

while(True):
    for filename in os.listdir(directory_path):
        filename_no_ext, _ = os.path.splitext(filename)
            
        # Check if the file has an image extension
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".gif")):                           
            # Get the full path of the image file
            image_path = os.path.join(directory_path, filename)

            image = cv2.imread(image_path)
            if image is not None:
                height, width, _ = image.shape
           
                #resized_image = cv2.resize(image, (height//4, width//4))
                #height, width, _ = resized_image.shape 
                #cv2.imwrite(directory_path + '/' + filename + '_resized.jpg', resized_image)
                detect(detection_graph, image_path, False)    
                #os.remove(directory_path + '/' + filename + '_resized.jpg')
                
                # Remove image
                os.remove(image_path)

