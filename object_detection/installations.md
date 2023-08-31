```
conda env export > environment.yml # We used this command in order to export conda environment
```

In order to replicate conda environment execute:
```
conda env create --name myenv --file=environment.yml
```

For Garbage detection model we need to do a few more steps:
- Inside the folder ~/anaconda3/envs/myenv/lib/python3.11/site-packages/object_detection/protos put the file string_int_label_map_pb2.py, this file is inside Jaco_Robotic_Arm/object_detection folder
- In the file ~/anaconda3/envs/myenv/lib/python3.11/site-packages/object_detection/utils/label_map_util.py, replace tf.gfile with tf.io.gfile, in line 132

For object detection scripts you need the annotations.json for garbage detection and the pretrained models. We put these required files, into two folders (Garbage Detection and Grape Leaves disease detection) inside Jaco_Robotic_Arm/object_detection folder. We have not included these two folders in our repo.\
You can find these folders at the following link: https://drive.google.com/drive/folders/1ws8gC-mo_Njot1KxvYj8udG0Ot2QC87c?usp=sharing Just put these folders into Jaco_Robotic_Arm/object_detection folder
