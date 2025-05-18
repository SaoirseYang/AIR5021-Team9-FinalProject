#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import os
import sys
import time
from PIL import Image, ImageDraw, ImageFont
import rospy
from spark_carry_object.msg import *
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError

class HSVDetection:
    def __init__(self):
        self.name = rospy.get_param("/spark_hsv_detection/color")
        self.image_topic = rospy.get_param("/spark_hsv_detection/image_topic", "/camera/rgb/image_filtered")
        self.bridge = CvBridge()
        self.HSV_value = [0, 0, 0]
        self.count = 0
        self.img = None
        self.image_received = False
        
        # 颜色块中心点上下左右扩大60个像素
        self.box_w = 60
        # 吸盘上方颜色像素范围
        self.cali_w = 20
        self.cali_h = 30
        # 收集300次取平均值
        self.collect_times = 300
        
        # Subscribe to the filtered image topic
        self.image_sub = rospy.Subscriber(self.image_topic, RosImage, self.image_callback, queue_size=1)
        
        # Call arm initialization
        self.arm_init()
        rospy.sleep(3)
        
        # Start HSV detection process
        rospy.Timer(rospy.Duration(0.05), self.process)  # 20Hz processing rate
    
    def image_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
    
    def arm_init(self):
        pub1 = rospy.Publisher('position_write_topic', position, queue_size=1)
        r1 = rospy.Rate(1)
        r1.sleep()
        pos = position()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        pub1.publish(pos)
        r1.sleep()
    
    def mean_hsv(self, img):
        HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.HSV_value[0] += np.mean(HSV[:, :, 0])
        self.HSV_value[1] += np.mean(HSV[:, :, 1])
        self.HSV_value[2] += np.mean(HSV[:, :, 2])
    
    def hsv_range(self):
        # 设置HSV颜色值的范围
        H_range = 12
        S_range = 120
        V_range = 120

        lower_H = int(self.HSV_value[0] - H_range)
        upper_H = int(self.HSV_value[0] + H_range)

        lower_S = int(self.HSV_value[1] - S_range)
        upper_S = int(self.HSV_value[1] + S_range)

        lower_V = int(self.HSV_value[2] - V_range)
        upper_V = int(self.HSV_value[2] + V_range)

        if lower_H < 0:
            lower_H = 0
        if upper_H > 180:
            upper_H = 180

        if lower_S < 50:
            lower_S = 50
        if upper_S > 255:
            upper_S = 255

        if lower_V < 50:
            lower_V = 50
        if upper_V > 255:
            upper_V = 255

        lower_HSV = np.array([lower_H, lower_S, lower_V])
        upper_HSV = np.array([upper_H, upper_S, upper_V])
        return lower_HSV, upper_HSV
    
    def test(self, lower_HSV, upper_HSV, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_HSV, upper_HSV)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.GaussianBlur(mask, (3, 3), 0)
        cv2.putText(mask, 'Done! Press q to exit!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("HSV_img", mask)
    
    def save_hsv(self, lower_HSV, upper_HSV):
        content = str(lower_HSV[0]) + ',' + str(lower_HSV[1]) + ',' + str(lower_HSV[2]) \
                  + ' ' + str(upper_HSV[0]) + ',' + str(upper_HSV[1]) + ',' + str(upper_HSV[2]) + '\n'
        # 将HSV值写入文件，文件在spark目录下
        if self.name == 'color_block':
            content = "block_HSV is :" + content
            filename = os.environ['HOME'] + "/color_block_HSV.txt"
        elif self.name == 'calibration':
            content = "calibration_HSV is :" + content
            filename = os.environ['HOME'] + "/calibration_HSV.txt"

        with open(filename, "w") as f:
            f.write(content)

        print("HSV value has saved in " + filename)
        print(content)
    
    def process(self, event=None):
        if not self.image_received:
            return
            
        if self.img is not None:
            # 200次以内先做提醒，将颜色块放在矩形框中
            if self.count < 200:
                cv2.putText(self.img, 'please put the color being', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                            (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(self.img, 'tested in rectangle box!', (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                            (0, 255, 0), 2, cv2.LINE_AA)
            # 如果在200-500次以内开始收集hsv值，并求出平均值
            elif self.count > 200 and self.count < 200 + self.collect_times:
                cv2.putText(self.img, 'HSV_value is collecting!', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                            (0, 255, 0), 2, cv2.LINE_AA)
                if self.name == 'color_block':
                    frame = self.img[120:120 + self.box_w, 300:300 + self.box_w]
                elif self.name == 'calibration':
                    frame = self.img[310:310 + self.cali_h, 355:355 + self.cali_w]
                self.mean_hsv(frame)
            # 500次以后开始检测，查看是否有提取到矩形框中颜色的HSV值
            elif self.count == 200 + self.collect_times:
                for i in range(len(self.HSV_value)):
                    self.HSV_value[i] = self.HSV_value[i] / self.collect_times
                lower_HSV, upper_HSV = self.hsv_range()
                self.save_hsv(lower_HSV, upper_HSV)
                self.lower_HSV = lower_HSV
                self.upper_HSV = upper_HSV

            elif self.count > 200 + self.collect_times:
                self.test(self.lower_HSV, self.upper_HSV, self.img)

            self.count += 1
            if self.name == 'color_block':
                cv2.rectangle(self.img, (300, 120), (300 + self.box_w, 120 + self.box_w), (0, 255, 0), 3)
            elif self.name == 'calibration':
                cv2.rectangle(self.img, (355, 310), (355 + self.cali_w, 310 + self.cali_h), (0, 255, 0), 3)
            cv2.imshow("RGB_img", self.img)
            if cv2.waitKey(10) == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

def main():
    try:
        rospy.init_node('hsv_detection', anonymous=True)
        hsv_detector = HSVDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()