#!/usr/bin/env python3

import os
import json
import csv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
from PIL import Image as PilImage
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster

from std_srvs.srv import Empty, EmptyResponse
from panoptic_mapping_msgs.msg import DetectronLabel, DetectronLabels


class FlatDataPlayer(Node):

    def __init__(self):
        """  Initialize ros node and read params """
        super().__init__('flat_data_player')
        # params
        self.declare_parameter(
            '~data_path', '/home/lukas/Documents/Datasets/flat_dataset/run1')
        self.data_path = self.get_parameter('~data_path').value

        self.declare_parameter('~global_frame_name', 'world')
        self.global_frame_name = self.get_parameter('~global_frame_name').value

        self.declare_parameter('~sensor_frame_name', 'depth_cam')
        self.sensor_frame_name = self.get_parameter('~sensor_frame_name').value

        self.declare_parameter('~use_detectron', False)
        self.use_detectron = self.get_parameter('~use_detectron').value

        self.declare_parameter('~play_rate', 1.0)
        self.play_rate = self.get_parameter('~play_rate').value

        self.declare_parameter('~wait', False)
        self.wait = self.get_parameter('~wait').value

        self.declare_parameter('~max_frames', 1e9)
        self.max_frames = self.get_parameter('~max_frames').value

        self.refresh_rate = 100  # Hz

        self.declare_parameter(
            '~static_transform',
            "1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1")
        static_transform = self.get_parameter(
            '~static_transform').value.replace(" ", "").split(",")
        self.static_transform = [float(x) for x in static_transform]
        self.get_logger().info(f"static_transform: {self.static_transform}")
        self.static_transform = np.array(self.static_transform).reshape(4, 4)

        self.declare_parameter('~add_labels_name', True)
        self.add_labels_name = self.get_parameter('~add_labels_name').value

        self.declare_parameter(
            '~labels_cvs_path',
            '/mnt/data/3d-lidar/semantic/self_collect/realsense_labels.csv')
        self.labels_cvs_path = self.get_parameter('~labels_cvs_path').value

        # ROS 2 Publishers
        self.color_pub = self.create_publisher(Image, '~/color_image', 100)
        self.depth_pub = self.create_publisher(Image, '~/depth_image', 100)
        self.id_pub = self.create_publisher(Image, '~/id_image', 100)
        if self.use_detectron:
            self.label_pub = self.create_publisher(DetectronLabels, '~/labels',
                                                   100)
        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 100)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Setup
        self.cv_bridge = CvBridge()
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.times = []
        self.ids = []
        self.current_index = 0  # Used to iterate through

        if not os.path.isfile(stamps_file):
            self.get_logger().fatal(
                f"No timestamp file '{stamps_file}' found.")
            raise FileNotFoundError(
                f"No timestamp file '{stamps_file}' found.")

        with open(stamps_file, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        # Load category_id -> category_name mapping
        self.category_id_to_category_name = {}
        with open(self.labels_cvs_path, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "InstanceID":
                    continue
                category_id = int(row[1])
                category_name = row[-2]
                self.category_id_to_category_name[category_id] = category_name

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)
        self.times = [(x - self.times[0]) / self.play_rate for x in self.times]
        self.start_time = None

        if self.wait:
            self.start_srv = self.create_service(Empty, '~/start',
                                                 self.start)
        else:
            self.start(None)

    def start(self, _):
        self.running = True
        self.timer = self.create_timer(1.0 / self.refresh_rate, self.callback)
        return Empty.Response()

    def callback(self):
        # Check we should be publishing.
        if not self.running:
            return

        # Check if done.
        if self.current_index >= len(self.times):
            self.get_logger().info("Finished playing the dataset.")
            rclpy.shutdown()
            return

        now = self.get_clock().now().to_msg()

        if self.start_time is None:
            self.start_time = now

        if self.times[self.current_index] > (now.sec + now.nanosec * 1e-9 - self.start_time.sec - self.start_time.nanosec * 1e-9):
            return

        # Get all data and publish.
        file_id = os.path.join(self.data_path, self.ids[self.current_index])

        # File paths
        color_file = file_id + "_color.png"
        depth_file = file_id + "_depth.tiff"
        pose_file = file_id + "_pose.txt"
        files = [color_file, depth_file, pose_file]

        if self.use_detectron:
            pred_file = file_id + "_predicted.png"
            labels_file = file_id + "_labels.json"
            files += [pred_file, labels_file]
        else:
            pred_file = file_id + "_segmentation.png"
            files.append(pred_file)

        for f in files:
            if not os.path.isfile(f):
                self.get_logger().warn(f"Could not find file '{f}', skipping frame.")
                self.current_index += 1
                return

        # Publish Color image
        cv_img = cv2.imread(color_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.color_pub.publish(img_msg)

        # Publish ID image
        cv_img = cv2.imread(pred_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img[:, :, 0], "8UC1")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.id_pub.publish(img_msg)

        # Publish labels
        if self.use_detectron:
            label_msg = DetectronLabels()
            label_msg.header.stamp = now
            try:
                with open(labels_file) as json_file:
                    data = json.load(json_file)
                    for d in data:
                        if 'instance_id' not in d:
                            d['instance_id'] = 0
                        if 'score' not in d:
                            d['score'] = 0
                        label = DetectronLabel()
                        label.id = d['id']
                        label.instance_id = d['instance_id']
                        label.is_thing = d['isthing']
                        label.category_id = d['category_id']
                        label.score = d['score']
                        label.category_name = self.category_id_to_category_name[d['category_id']]
                        label_msg.labels.append(label)
                self.label_pub.publish(label_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to load labels: {str(e)}")

        # Publish Depth image
        cv_img = PilImage.open(depth_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(cv_img), "32FC1")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.sensor_frame_name
        self.depth_pub.publish(img_msg)

        # Publish transform
        if os.path.isfile(pose_file):
            pose_data = [float(x) for x in open(pose_file, 'r').read().split()]
            transform = np.eye(4)
            for row in range(4):
                for col in range(4):
                    transform[row, col] = pose_data[row * 4 + col]
            transform = self.static_transform @ transform
            rotation = tf2_ros.transformations.quaternion_from_matrix(transform)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.global_frame_name
            t.child_frame_id = self.sensor_frame_name
            t.transform.translation.x = transform[0, 3]
            t.transform.translation.y = transform[1, 3]
            t.transform.translation.z = transform[2, 3]
            t.transform.rotation.x = rotation[0]
            t.transform.rotation.y = rotation[1]
            t.transform.rotation.z = rotation[2]
            t.transform.rotation.w = rotation[3]
            self.tf_broadcaster.sendTransform(t)

            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.global_frame_name
            pose_msg.pose.position.x = pose_data[3]
            pose_msg.pose.position.y = pose_data[7]
            pose_msg.pose.position.z = pose_data[11]
            pose_msg.pose.orientation.x = rotation[0]
            pose_msg.pose.orientation.y = rotation[1]
            pose_msg.pose.orientation.z = rotation[2]
            pose_msg.pose.orientation.w = rotation[3]
            self.pose_pub.publish(pose_msg)

        self.current_index += 1
        if self.current_index > self.max_frames:
            self.get_logger().info(f"Played reached max frames ({self.max_frames})")
            rclpy.shutdown()

def main(args=None):
    rclpy.init()
    player = FlatDataPlayer()
    try:
        rclpy.spin(player)
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
