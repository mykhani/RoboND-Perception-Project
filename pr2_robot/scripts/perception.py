#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

class dropbox:
    def __init__(self):
        self.name = ""
        self.group = ""
        self.position = []
    
# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    pcl_input_pub.publish(pcl_msg)
# Exercise-2 TODOs:
    world_cloud = ros_to_pcl(pcl_msg)

    # TODO: Convert ROS msg to PCL data
    cloud = world_cloud
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    pcl_voxel_filter_pub.publish(pcl_to_ros(cloud_filtered))
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
    pcl_noise_filtered_pub.publish(pcl_to_ros(cloud_filtered))

    
    # TODO: PassThrough Filter
    
    # Cut off along z-axis
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    z_axis_min = 0.6
    z_axis_max = 1.0
    passthrough.set_filter_limits(z_axis_min, z_axis_max)
    cloud_filtered = passthrough.filter()
    pcl_passthroughz_pub.publish(pcl_to_ros(cloud_filtered))
    
    # Cut off along y-axis
    filter_axis = 'y'
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    y_axis_min = -0.5
    y_axis_max = 0.5
    passthrough.set_filter_limits(y_axis_min, y_axis_max)
    cloud_filtered = passthrough.filter()
    pcl_passthroughy_pub.publish(pcl_to_ros(cloud_filtered))

    # Cut off along x-axis
    filter_axis = 'x'
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    x_axis_min = 0
    x_axis_max = 1.0
    passthrough.set_filter_limits(x_axis_min, x_axis_max)
    cloud_filtered = passthrough.filter()

    pcl_filtered_pub.publish(pcl_to_ros(cloud_filtered)) 
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
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)# Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    print("Clusters detected: %d" %len(cluster_indices))
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)

    # TODO: Publish ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert point cloud cluster to ros message
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Get features
        color_hist = compute_color_histograms(ros_cluster, using_hsv = True)
        normal_hist = compute_normal_histograms(get_normals(ros_cluster))
        
        # Compute the associated feature vector
        features = np.concatenate((color_hist, normal_hist)) 

        # Make the prediction
        # Predict and get label
        prediction = clf.predict(scaler.transform(features.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
        
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    
    scene_num = rospy.get_param('/scene_num')
    print("Current Scene: %d" %scene_num)

    # TODO: Parse parameters into individual variables
    object_names = []
    object_groups = []

    for object in object_list_param:
        object_names.append(object['name'])
        object_groups.append(object['group'])

    boxes = []
    for param in dropbox_param:
        box = dropbox()
        box.name = param['name']
        box.group = param['group']
        box.position = param['position']
        boxes.append(box)

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    dict_list = []
    # TODO: Loop through the pick list
    for index, object_name in enumerate(object_names):
        object_found = False
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        for detected_object in object_list:
            if detected_object.label == object_name:
                object_found = True
                cloud = detected_object.cloud
                break

        if not object_found:
            print("Failed to find picklist item: %s" %object_name)
            continue

        # Calculate centroid
        points_arr = ros_to_pcl(cloud).to_array()
        x, y, z = np.mean(points_arr, axis=0)[:3]
        # convert to native Python float datatype
        x = np.asscalar(x)
        y = np.asscalar(y)
        z = np.asscalar(z)
        # TODO: Create 'place_pose' for the object
        pick_pose = Pose()
        pick_pose.position.x = x
        pick_pose.position.y = y
        pick_pose.position.z = z

        # TODO: Assign the arm to be used for pick_place
        object_group = object_groups[index]

        arm_name = ""
        place_pose = Pose()

        for box in boxes:
            if box.group == object_group:
                arm_name = box.name
                x, y, z = box.position
                place_pose.position.x = x
                place_pose.position.y = y
                place_pose.position.z = z
                
        print("Selected arm: %s" %arm_name)

        # create ROS service request variables
        msg_scene_num = Int32()
        msg_scene_num.data = scene_num

        msg_arm_name = String()
        msg_arm_name.data = arm_name
        
        msg_object_name = String()
        msg_object_name.data = object_name
        
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(msg_scene_num, msg_arm_name, msg_object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
        
        
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            # Create ros service messages
            print("Calling pick and place server")
            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(msg_scene_num, msg_object_name, msg_arm_name, pick_pose, place_pose)

            #print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yaml_filename = 'output_' + str(scene_num) + '.yaml'
    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    pr2_command_pub = rospy.Publisher("/pr2/world_joint_controller/command", PointCloud2, queue_size=1)
    pcl_input_pub = rospy.Publisher("/pcl_input_data", PointCloud2, queue_size=1)
    pcl_voxel_filter_pub = rospy.Publisher("/pcl_voxel_filtered_data", PointCloud2, queue_size=1)
    pcl_noise_filtered_pub = rospy.Publisher("/pcl_noise_filtered_data", PointCloud2, queue_size=1)
    pcl_filtered_pub = rospy.Publisher("/pcl_filtered_data", PointCloud2, queue_size=1)
    pcl_passthroughz_pub = rospy.Publisher("/pcl_passthroughz_data", PointCloud2, queue_size=1)
    pcl_passthroughy_pub = rospy.Publisher("/pcl_passthroughy_data", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)
    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
