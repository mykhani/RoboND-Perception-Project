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
To select a world scene, edit the pick_place_project.launch file and change the parameters accordingly.
Below is the snapshot of the launch file settings that need to be modified to select the world scene 1,2 and 3.

![alt text][selecting_the_scene]

#### 2. Voxel Grid Downsampling of the Point Cloud Data
The computational requirement can be reduced and processing can be performed faster by reducing the number of points in point cloud in such way that they still retain useful information. This can be performed by voxel grid filtering. Below is the relevant code for voxel grid filtering.

``` python
vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
```
Below is the data after voxel grid downsampling.

![alt text][voxel_filtered]
#### 3. Filtering the Noise from input camera data
Below is the image of unfiltered raw camera input data.

![alt text][unfiltered_input_data]

Below is the image after applying statistical outlier filter to remove noise grains.  

![alt text][filtered_input_data]

Here is the code for statistical outlier filter.

```python
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)

    # Set threshold scale factor
    x = 0.015625

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
```

The number of points to analyze and standard deviation fator were selected based on trial and error process. To guide the selection process, the following observations were considered:
* The number of points cannot be very larger as to oversize the target objects themselves nor too small to have no effect.
* There was significant distance between noise particles. Thus lowering the standard deviation threshold should effectively filter out the noise particles.

#### 4. Focusing on the regions of interest via passthrough filtering
The point cloud contains many redundant regions that are not useful for our processing and it's better to get rid of them. For this, I have used the passthrough filter. This filter allows to limit the cloud points between minimum and maximum cutoff points along a certain axis.

Below is the data before passthrough filtering.

![alt text][before_passthrough]

Below is the data after applying passthrough filter along z-axis.

![alt text][after_z_passthrough]

Here's the relevant code.

```python
# Cut off along z-axis
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    z_axis_min = 0.6
    z_axis_max = 1.0
    passthrough.set_filter_limits(z_axis_min, z_axis_max)
    cloud_filtered = passthrough.filter()
```

Below is the data after applying passthrough filter along y-axis.

![alt text][after_y_passthrough]

Here's the relevant code.

```python
# Cut off along y-axis
    filter_axis = 'y'
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    y_axis_min = -0.5
    y_axis_max = 0.5
    passthrough.set_filter_limits(y_axis_min, y_axis_max)
    cloud_filtered = passthrough.filter()
```

#### 5. RANSAC Segmentation
Random Sample Consensus algorithm is used to detect the table since table can be easily modeled as a plane. The RANSAN algorithm, when applied on point cloud data gives the indices of the table points which can then be used to extract inliers (table) and outliers (objects).  

Below is the image of inliers after applying RANSAC segmentation.

![alt text][ransac_inlier]

Below is the image of outliers after applying RANSAC segmentation.

![alt text][ransac_outlier]

Here's the code.

```python
    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
```

For our analysis, we are interested in the outliers i.e. objects placed on table.

#### 6. DBSCAN or Euclidean Clustering
After extracting the outliers, we need to compute which points belong to a certain object in the point cloud. Since objects are placed at considerable distance apart, intuitively we know that the point that belong to a same object must lie very close together, as a cluster. To find out these points, DBSCAN which stands for Density Based Spatial Clustering for Applications with Noise. As the name implies, DBSCAN searches for points that lie close together, meeting a specific criteria for closeness. Below are the results of clustering performed on all three worlds.

Clustering for world scene 1.

![alt text][segmentation_and_clustering_1]

Clustering for world scene 2.

![alt text][segmentation_and_clustering_2]

Clustering for world scene 3.

![alt text][segmentation_and_clustering_3]

Here's the relevant code.

```python
    c = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
```

In the above code, the tolerance is the closeness criteria and the lower this value, the closer are points. It is worth noting that since we have used voxel grid filtering with grid size of 0.01, this tolerance value cannot be less than 0.01. We also set the minimum and maximum cluster size which rejects points that meet the closeness criteria but are less in number.

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
* [output_1.yaml](https://github.com/mykhani/RoboND-Perception-Project/blob/master/output_1.yaml)
* [output_2.yaml](https://github.com/mykhani/RoboND-Perception-Project/blob/master/output_2.yaml)
* [output_3.yaml](https://github.com/mykhani/RoboND-Perception-Project/blob/master/output_3.yaml)

#### 2. SVM Model and Training Data
* [SVM Model](https://github.com/mykhani/RoboND-Perception-Project/blob/master/model.sav)
* [SVM Training Data](https://github.com/mykhani/RoboND-Perception-Project/blob/master/training_set.sav)

### Limitations & Future work
* I have observed some strange behavior where the arm moves to the correct location but is unable to grasp objects. I will investigate this behavior in future.
* With "Linear" kernel for SVM training, I could manage to obtain only 90% accuracy. I will investigate into how to improve it further.
* I will try to add my own publishing of 3D collision map cloud for planning out arm movment without collisions.
