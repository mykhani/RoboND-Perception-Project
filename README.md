[//]: # (Image References)
[selecting_the_scene]: ./images/selecting_the_scene.png
[unfiltered_input_data]: ./images/unfiltered_input_data.png
[filtered_input_data]: ./images/filtered_input_data.png
[voxel_filtered]: ./images/voxel_filtered_data.png
[before_passthrough]: ./images/before_passthrough_filter.png
[after_z_passthrough]: ./images/after_passthrough_z_axis.png
[after_y_passthrough]: ./images/after_passthrough_y_axis.png
[ransac_inlier]: ./images/ransac_inlier.png
[ransac_outlier]: ./images/ransac_outlier.png
[segmentation_and_clustering_1]: ./images/segmentation_and_clustering_1.png
[segmentation_and_clustering_2]: ./images/segmentation_and_clustering_2.png
[segmentation_and_clustering_3]: ./images/segmentation_and_clustering_3.png
[capturing_features]: ./images/capturing_features.jpg
[training_results]: ./images/training_results.png
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

#### 2. Voxel Grid Downsampling of the Point Cloud Data
Below is the data after voxel grid downsampling.

![alt text][voxel_filtered]
#### 3. Filtering the Noise from input camera data
Below is the image of unfiltered raw camera input data.

![alt text][unfiltered_input_data]

Below is the image after applying statistical outlier filter to remove noise grains.

![alt text][filtered_input_data]

#### 4. Focusing on the regions of interest via passthrough filtering
Below is the data before passthrough filtering.

![alt text][before_passthrough]

Below is the data after applying passthrough filter along z-axis.

![alt text][after_z_passthrough]

Below is the data after applying passthrough filter along y-axis.

![alt text][after_y_passthrough]

#### 5. RANSAC Segmentation
Below is the image of inliers after applying RANSAC segmentation.

![alt text][ransac_inlier]

Below is the image of outliers after applying RANSAC segmentation.

![alt text][ransac_outlier]

#### 6. DBSCAN or Euclidean Clustering
Clustering for world scene 1.

![alt text][segmentation_and_clustering_1]

Clustering for world scene 2.

![alt text][segmentation_and_clustering_2]

Clustering for world scene 3.

![alt text][segmentation_and_clustering_3]

#### 7. Feature extraction and training the SVM
```bash
robond@udacity:~$ roslaunch sensor_stick training.launch
```

Below screenshot shows capturing process underway.
![alt text][capturing_features]

Below are the results of SVM training.

![alt text][training_results]

#### 8. Identifying the objects
Predictions for world scene 1

![alt text][prediction_1]

Predictions for world scene 2

![alt text][prediction_2]

Predictions for world scene 3

![alt text][prediction_3]

### Output Files
#### 1. Output YAML files
[output_1.yaml](https://github.com/mykhani/RoboND-Perception-Project/blob/master/output_1.yaml)
[output_2.yaml](https://github.com/mykhani/RoboND-Perception-Project/blob/master/output_2.yaml)
[output_3.yaml](https://github.com/mykhani/RoboND-Perception-Project/blob/master/output_3.yaml)

#### 2. SVM Model and Training Data
[SVM Model](https://github.com/mykhani/RoboND-Perception-Project/blob/master/model.sav)
[SVM Training Data](https://github.com/mykhani/RoboND-Perception-Project/blob/master/training_set.sav)

### Limitations & Future work
* I have observed some strange behavior where the arm moves to the correct location but is unable to grasp objects. I will investigate this behavior in future.
* With "Linear" kernel for SVM training, I could manage to obtain only 90% accuracy. I will investigate into how to improve it further.
* I will try to add my own publishing of 3D collision map cloud for planning out arm movment without collisions.
