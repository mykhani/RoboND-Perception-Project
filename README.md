[//]: # (Image References)
[selecting_the_scene]: ./images/selecting_the_scene.png
[unfiltered_input_data]: ./images/unfiltered_input_data.png
[filtered_input_data]: ./images/filtered_input_data.png
[segmentation_and_clustering_1]: ./images/segmentation_and_clustering_1.png
[segmentation_and_clustering_2]: ./images/segmentation_and_clustering_2.png
[segmentation_and_clustering_3]: ./images/segmentation_and_clustering_1.png
[prediction_1]: ./images/prediction_1.png
[prediction_2]: ./images/prediction_2.png
[prediction_3]: ./images/prediction_3.png

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# 3D Perception Solution
Please follow this link for detailed steps for setting up the environment: [link](https://github.com/udacity/RoboND-Perception-Project/blob/master/README.md)

### Implementing Image Processing Pipeline
#### 1. Selecting the world scene
Below is the snapshot of the launch file settings that need to be modified to select the world scene 1,2 and 3.
![alt text][selecting_the_scene]
#### 2. Voxel Downsampling of the Point Cloud Data
#### 3. Filtering the Noise from input camera data
Below is the image of unfiltered raw camera input data.
![alt text][unfiltered_input_data]
Below is the image after applying statistical outlier filter to remove noise grains.
![alt text][filtered_input_data]
#### 4. Focusing on the regions of interest via passthrough filtering
#### 5. RANSAC Segmentation
#### 6. DBSCAN or Euclidean Clustering
Clustering for world scene 1.
![alt text][segmentation_and_clustering_1]
Clustering for world scene 2.
![alt text][segmentation_and_clustering_2]
Clustering for world scene 3.
![alt text][segmentation_and_clustering_3]
#### 7. Feature extraction and training the SVM
#### 8. Identifying the objects
Predictions for world scene 1
![alt text][prediction_1]
Predictions for world scene 2
![alt text][prediction_2]
Predictions for world scene 3
![alt text][prediction_3]
### Generating the output commands for Pick and Place service
