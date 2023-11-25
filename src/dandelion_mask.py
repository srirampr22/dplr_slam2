#!/usr/bin/env python3
import os
import sys
import threading
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, PointField

RGB_TOPIC = '/camera/color/image_raw'
YELLOW_HSV_LOWER = np.array([20, 105, 122])
YELLOW_HSV_UPPER = np.array([33, 255, 255])

class YellowMaskGeneratorNode(object):
    def __init__(self):
        self._rgb_input_topic = RGB_TOPIC
        self._publish_rate = rospy.get_param('~publish_rate', 100)

        self._last_msg = None
        self._msg_lock = threading.Lock()

        self._yellow_mask_pub = rospy.Publisher('/yellow_mask', Image, queue_size=1)
        self._mean_point_pub = rospy.Publisher('/mean_point_mask', Image,queue_size=1)

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

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if msg is not None:
                np_image = self.imgmsg_to_cv2(msg)

                # Convert RGB image to HSV for color filtering
                hsv_image = cv2.cvtColor(np_image, cv2.COLOR_BGR2HSV)

                # Create a binary mask for yellow pixels based on the color range
                yellow_mask = cv2.inRange(hsv_image, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)

                # Finding contours
                contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Filter contours
                threshold = 500
                circularity_threshold = 0.2  # Adjust based on how circular you want the shape to be. 1 is a perfect circle.
                large_circular_contours = []
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter == 0:
                        continue
                    circularity = 4 * np.pi * area / (perimeter ** 2)
                    if area > threshold and circularity > circularity_threshold:
                        large_circular_contours.append(cnt)

                # Create an empty mask
                large_contours_mask = np.zeros_like(yellow_mask)

                # Draw the large and circular contours on the empty mask
                cv2.drawContours(large_contours_mask, large_circular_contours, -1, (255), thickness=-1)

                # Create an output image with only the large-contour yellow pixels
                yellow_large_contour_image = cv2.bitwise_and(np_image, np_image, mask=large_contours_mask)

                # Draw the large and circular contours with a green outline on the image
                green_color = (0, 255, 0)  # RGB color for green
                pink_color = (0, 0, 255) 
                mean_point_mask = np.zeros_like(np_image)

                cv2.drawContours(yellow_large_contour_image, large_circular_contours, -1, green_color, thickness=2)

                # Draw the center mean point for each contour
                dot_radius = 9  # Adjust as necessary
                for cnt in large_circular_contours:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(yellow_large_contour_image, (cx, cy), dot_radius, pink_color, -1)
                        cv2.circle(mean_point_mask, (cx, cy), dot_radius, pink_color, -1)

                # Publish the mask as a new topic
                print("IMAGE RECIVED")

                mean_point_msg = self.cv2_to_imgmsg(mean_point_mask)
                self._mean_point_pub.publish(mean_point_msg)

                result_msg = self.cv2_to_imgmsg(yellow_large_contour_image)
                self._yellow_mask_pub.publish(result_msg)
                
            else:
                print("No image recived")

        rate.sleep()

    def _image_callback(self, msg):
        self._msg_lock.acquire()
        self._last_msg = msg
        self._msg_lock.release()

def main():
    rospy.init_node('yellow_mask_generator')

    node = YellowMaskGeneratorNode()
    node.run()

if __name__ == '__main__':
    main()


