#!/usr/bin/env python3
import rospy
import numpy as np
from tqdm import tqdm
import random
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import std_msgs
import ros_numpy
import message_filters
import tf.transformations as tr
import pdb
import ctypes
import struct
from sklearn.cluster import DBSCAN
from sklearn.mixture import GaussianMixture
import time
from tf2_msgs.msg import TFMessage
from ros_numpy.point_cloud2 import pointcloud2_to_array, array_to_pointcloud2, split_rgb_field, merge_rgb_fields
from datetime import datetime
from scipy.spatial.distance import euclidean, cdist
import colorsys
from visualization_msgs.msg import Marker, MarkerArray
import zmq
import json


class PointcloudProcessor():
    def __init__(self):
        rospy.init_node('pcl_processor', anonymous=True)
        rospy.Subscriber('/map', PointCloud2, self.pcl_cluster_meancov)

        self.pclpublisher = rospy.Publisher(
            'dandelion_map', PointCloud2, latch=True, queue_size=100)
        self.color_filter_publisher = rospy.Publisher(
            'color_filter__map', PointCloud2, latch=True, queue_size=100)
        self.marker_publisher = rospy.Publisher(
            'cluster_labels', MarkerArray, queue_size=10)

        self.pcl_buffer = {}
        self.true_buffer = {}
        # self.accumulated_points = []
        # self.accumulated_label = []
        self.assigned_labels = []

        # ZMQ publisher
        zmq_context = zmq.Context()
        self.zmq_publisher = zmq_context.socket(zmq.PUB)
        self.zmq_publisher.bind("tcp://127.0.0.1:5556")

    def publish_zmq_message(self, message):
        # Convert numpy arrays to Python lists
        for key in message:
            message[key] = message[key].tolist()
            
        json_message = json.dumps(message)
        self.zmq_publisher.send_string(json_message)

    def pcl_cluster_meancov(self, pcl_msg):

        pcl_xyzcol = pointcloud2_to_array(pcl_msg)
        pcl_mat = split_rgb_field(pcl_xyzcol)

        maskintcol1 = (pcl_mat['r'] == 0)
        maskintcol2 = (pcl_mat['g'] == 255)
        maskintcol3 = (pcl_mat['b'] == 0)

        mask_c = np.logical_and(np.logical_and(
            maskintcol1, maskintcol2), maskintcol3)
        pcl_mat = pcl_mat[mask_c]

        pcl_mat_xyzrgb = self.pcl_mat_to_XYZRGB(pcl_mat)

        filtered_pcl_mat = pcl_mat_xyzrgb

        filtered_unfiltered = self.XYZRGB_to_pcl_mat(filtered_pcl_mat)
        filtered_array_unfiltered = merge_rgb_fields(filtered_unfiltered)
        
        filtered_msg_unfiltered = ros_numpy.msgify(
            PointCloud2, filtered_array_unfiltered, stamp=pcl_msg.header.stamp, frame_id=pcl_msg.header.frame_id)
        self.color_filter_publisher.publish(filtered_msg_unfiltered)

        if (filtered_pcl_mat.shape[0] > 0):

            t0_dbscan = time.time()

            db = DBSCAN(eps=0.05, min_samples=2, n_jobs=-1)
            for _ in tqdm(range(10), desc='Clustering'):
                db.fit(filtered_pcl_mat[:, :3])

            print("DBSCAN computation time:", time.time() - t0_dbscan)

            labels = db.labels_
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            # will be n x 7 cols array
            xyz_cl = np.c_[filtered_pcl_mat, labels]
            # de-noises data
            # maskint2 = (xyz_cl[:, -1] == -1)
            # xyz_cl = xyz_cl[np.logical_not(maskint2)]

            if (xyz_cl.shape[0] > 0):

                DBSCAN_labels, DBSCAN_counts = np.unique(
                    xyz_cl[:, -1], return_counts=True)

                for label in DBSCAN_labels:
                    new_label = random.randint(0, 30)
                    while new_label in self.assigned_labels:
                        new_label = random.randint(0, 30)

                    xyz_cl[xyz_cl[:, -1] == label, -1] = new_label
                    self.assigned_labels.append(new_label)

                new_labels = np.unique(xyz_cl[:, -1])

                print("Newly assgined labels:", new_labels,
                      "xyz_cl shape:", xyz_cl.shape)

                cluster_mpos_dict = self.compute_cluster_stats(xyz_cl)

                # ######################################### BUFEER UPDATE##############################################
                buffer_entry = {}
                for label in new_labels:
                    if label in cluster_mpos_dict:
                        buffer_entry[label] = cluster_mpos_dict[label]['mean']

                # Store the buffer entry with timestamp (t = x)
                self.pcl_buffer[pcl_msg.header.stamp] = buffer_entry

                # Most recent previous timestamp (t = x-1)
                prev_time_stamp = None
                for timestamp in self.pcl_buffer.keys():
                    if timestamp < pcl_msg.header.stamp:
                        if prev_time_stamp is None or timestamp > prev_time_stamp:
                            prev_time_stamp = timestamp

                if prev_time_stamp is not None:

                    # prev is t-1
                    prev_true_buffer = self.true_buffer[prev_time_stamp]
                    curr_buffer_entry = self.pcl_buffer[pcl_msg.header.stamp]

                    new_true_dict = self.pos_comparission(
                        prev_true_buffer, curr_buffer_entry)
                    bucket = new_true_dict
                    true_flower_labels = list(new_true_dict.keys())

                else:

                    bucket = cluster_mpos_dict
                    true_flower_labels = list(cluster_mpos_dict.keys())

                ######################################################################################################

                true_flower_coordinates = {}

                print("true_flower_labels:", true_flower_labels)
                print("bucket:", bucket)

                for label in true_flower_labels:
                    true_flower_coordinates[label] = bucket[label]['mean']

                self.true_buffer[pcl_msg.header.stamp] = true_flower_coordinates

                # List to store the cube vertices
                bounding_cubes = []

                # Create a cube for each mean value
                for _, mean in true_flower_coordinates.items():
                    bounding_cubes.extend(self.create_cube_outline(mean))

                self.publish_cluster_labels(true_flower_coordinates)

                self.publish_zmq_message(true_flower_coordinates)

                # Convert the cube vertices to the same point cloud format
                bounding_cubes = np.array(bounding_cubes)
                pcl_cubes = np.zeros((len(bounding_cubes),),
                                     dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),
                                            ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)])
                pcl_cubes['x'] = bounding_cubes[:, 0]
                pcl_cubes['y'] = bounding_cubes[:, 1]
                pcl_cubes['z'] = bounding_cubes[:, 2]
                pcl_cubes['r'] = 255
                pcl_cubes['g'] = 0
                pcl_cubes['b'] = 127

                pcl_unfiltered = self.XYZRGB_to_pcl_mat(
                    xyz_cl)
                

                # Merge cube vertices with the original point cloud data
                pcl_with_cubes = np.concatenate([pcl_unfiltered, pcl_cubes])
                pc_array_unfiltered = merge_rgb_fields(pcl_with_cubes)
                pc_msg_unfiltered = ros_numpy.msgify(
                    PointCloud2, pc_array_unfiltered, stamp=pcl_msg.header.stamp, frame_id=pcl_msg.header.frame_id)
                self.pclpublisher.publish(pc_msg_unfiltered)
            else:
                print("No clusters found, DBSCAN did not find anything")
        else:
            print("No clusters found, No green dots")

    def compute_cluster_stats(self, xyz_cl):
        cluster_labels = np.unique(xyz_cl[:, -1])
        cluster_stats = {}

        for label in cluster_labels:
            if label == -1:
                continue  # Skip noise points

            cluster_points = xyz_cl[xyz_cl[:, -1] == label][:, :3]
            cluster_mean = np.mean(cluster_points, axis=0)
            # cluster_cov = np.cov(cluster_points, rowvar=False)

            cluster_stats[label] = {
                'mean': cluster_mean}

        return cluster_stats

    def pos_comparission(self, prev_true_buffer, curr_buffer_entry):
        true_cluster_data = list(prev_true_buffer.items())
        curr_cluster_data = list(curr_buffer_entry.items())

        # Extract labels and means separately:
        true_cluster_labels = [item[0] for item in true_cluster_data]
        true_cluster_means = [
            item[1]['mean'] if 'mean' in item[1] else item[1] for item in true_cluster_data]

        curr_cluster_labels = [item[0] for item in curr_cluster_data]
        curr_cluster_means = [
            item[1]['mean'] if 'mean' in item[1] else item[1] for item in curr_cluster_data]

        print("true_cluster_labels:", true_cluster_labels)
        print("true_cluster_means:", true_cluster_means)

        print("curr_cluster_labels", curr_cluster_labels)
        print("curr_cluster_means", curr_cluster_means)

        threshold = 0.05  # We can adjust this value based on your requirements

        updated_cluster_data = {}

        for i, curr_mean in enumerate(curr_cluster_means):
            distances = np.linalg.norm(
                np.array(true_cluster_means) - curr_mean, axis=1)
            closest_idx = np.argmin(distances)
            if distances[closest_idx] < threshold:
                updated_cluster_data[true_cluster_labels[closest_idx]] = {
                    'mean': curr_mean}
            else:
                updated_cluster_data[curr_cluster_labels[i]] = {
                    'mean': curr_mean}

        return updated_cluster_data

    def publish_cluster_labels(self, flower_coordinates):
        marker_array = MarkerArray()

        for index, (label, mean) in enumerate(flower_coordinates.items()):
            marker = Marker()
            marker.header.frame_id = "map"  # Use your frame here
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = mean[0]
            marker.pose.position.y = mean[1]
            # Adding a small offset in z direction to display above the cube
            marker.pose.position.z = mean[2] + 0.06
            marker.text = str(label)
            marker.scale.z = 0.04  # Size of the text
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def pcl_mat_to_XYZRGB(self, pcl_mat):
        # Extract color components
        red = pcl_mat['r']
        green = pcl_mat['g']
        blue = pcl_mat['b']

        # Create a new point cloud matrix with XYZRGB values
        xyzrgb = np.zeros((pcl_mat.shape[0], 6))
        xyzrgb[:, 0] = pcl_mat['x']
        xyzrgb[:, 1] = pcl_mat['y']
        xyzrgb[:, 2] = pcl_mat['z']
        xyzrgb[:, 3] = red
        xyzrgb[:, 4] = green
        xyzrgb[:, 5] = blue

        return xyzrgb

    def XYZRGB_to_pcl_mat(self, xyzrgb):
        # Extract XYZRGB components
        x = xyzrgb[:, 0]
        y = xyzrgb[:, 1]
        z = xyzrgb[:, 2]
        red = xyzrgb[:, 3]
        green = xyzrgb[:, 4]
        blue = xyzrgb[:, 5]

        # Create a new point cloud matrix with separate components
        pcl_mat = np.zeros((xyzrgb.shape[0],),
                           dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),
                                  ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)])
        pcl_mat['x'] = x
        pcl_mat['y'] = y
        pcl_mat['z'] = z
        pcl_mat['r'] = red
        pcl_mat['g'] = green
        pcl_mat['b'] = blue

        return pcl_mat

    def create_cube_outline(self, center, size=0.05):
        """
        Given a center point, compute the cube's outline centered at that point.
        :param center: (numpy array) 1x3 array containing XYZ coordinates of the center point.
        :param size: (float) side length of the cube.
        :return: (numpy array) Nx3 array containing XYZ coordinates of the cube's outline.
        """
        half_size = size / 2.0

        # Define all 8 vertices of the cube
        vertices = [
            center + np.array([half_size, half_size, half_size]),
            center + np.array([-half_size, half_size, half_size]),
            center + np.array([-half_size, -half_size, half_size]),
            center + np.array([half_size, -half_size, half_size]),
            center + np.array([half_size, half_size, -half_size]),
            center + np.array([-half_size, half_size, -half_size]),
            center + np.array([-half_size, -half_size, -half_size]),
            center + np.array([half_size, -half_size, -half_size]),
        ]

        # Define the 12 edges of the cube using pairs of vertices
        edges = [
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[3]),
            (vertices[3], vertices[0]),
            (vertices[4], vertices[5]),
            (vertices[5], vertices[6]),
            (vertices[6], vertices[7]),
            (vertices[7], vertices[4]),
            (vertices[0], vertices[4]),
            (vertices[1], vertices[5]),
            (vertices[2], vertices[6]),
            (vertices[3], vertices[7]),
        ]

        outline_points = []

        # For each edge, generate points to represent that edge
        for edge in edges:
            # The following code linearly interpolates between the start and end of each edge
            num_points = int(size / 0.0005)  # For every 5 mm
            for i in range(num_points + 1):
                point = edge[0] * (1 - i/num_points) + edge[1] * (i/num_points)
                outline_points.append(point)

        return np.array(outline_points)


if __name__ == '__main__':
    pcl_node = PointcloudProcessor()
    rospy.spin()
