#!/usr/bin/env python3
import os
import sys
# import threading
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
import pdb
from pupil_apriltags import Detector
import time
from argparse import ArgumentParser
sys.path.append('/home/sriram/ssl_slam2_ws/src/AprilTag/scripts')

import apriltag
print("success")

RGB_TOPIC = '/camera/color/image_raw'
YELLOW_HSV_LOWER = np.array([20, 105, 122])
YELLOW_HSV_UPPER = np.array([33, 255, 255])

class Tagdetector(object):
    def __init__(self):
        self._rgb_input_topic = RGB_TOPIC
        self._publish_rate = rospy.get_param('~publish_rate', 10)

        self._last_msg = None
        self.tag_pub = rospy.Publisher('/tag_detection', Image, queue_size=1)

    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                        dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        image_reverse = np.flip(image_opencv, axis=2)

        return image_reverse

    def cv2_to_imgmsg(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tobytes()
        img_msg.step = len(img_msg.data) // img_msg.height
        return img_msg

    def run(self):
        rospy.Subscriber(self._rgb_input_topic, Image,
                         self._image_callback, queue_size=1)

        # rate = rospy.Rate(self._publish_rate)
        rate = rospy.Rate(rospy.get_param('~hz',1))
        while not rospy.is_shutdown():

            msg = self._last_msg

            if msg is not None:
                np_image = self.imgmsg_to_cv2(msg)
                np_image = np_image.copy()  # Ensure the image is continuous

                if np_image is not None and np_image.size > 0:

                    ####################
                    # parser = ArgumentParser(description='Detect AprilTags from video stream.')
                    # apriltag.add_arguments(parser)
                    # options = parser.parse_args()

                    # detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())
                    ########################

                    gray_img = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)
                 
                    at_detector = Detector(families='tag16h5',nthreads=2)

                    K = ([594.5518798828125/1000, 594.532958984375/1000, 327.4517517089844/1000, 240.4869384765625/1000])

                    result =  at_detector.detect(gray_img,estimate_tag_pose=False, camera_params=K, tag_size=0.147)

                    print(result)
                    print("DEBUG checkpoint1")
                    sys.stdout.flush()

                    # if !(result.empty()
                    for tag in result:
                        print("DEBUG checkpoint2")
                        # pdb.set_trace()
                        if tag.tag_id == 0:
                            corners = tag.corners

                            if len(corners) == 4:
                                for i in range(4):
                                    pt1 = tuple(corners[i].astype(int))
                                    pt2 = tuple(corners[(i+1) % 4].astype(int))
                                    cv2.line(np_image, pt1, pt2, (0, 255, 0), 2)
                                cv2.putText(np_image, str(tag.tag_id), 
                                            (int(tag.center[0]), int(tag.center[1])), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    cv2.imshow("AprilTag Detections", np_image)
                    cv2.waitKey(500)

        rate.sleep()

    def _image_callback(self, msg):
        # self._msg_lock.acquire()
        self._last_msg = msg
        # self._msg_lock.release()

def main():
    rospy.init_node('tag_detector')

    node = Tagdetector()
    node.run()

if __name__ == '__main__':
    main()


