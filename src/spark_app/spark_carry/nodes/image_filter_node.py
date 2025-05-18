#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageFilterNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('image_filter_node', anonymous=True)
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 从参数服务器获取参数
        self.b_gain = rospy.get_param('~b_gain', 1.2)
        self.g_gain = rospy.get_param('~g_gain', 0.85)
        self.r_gain = rospy.get_param('~r_gain', 0.95)
        self.contrast = rospy.get_param('~contrast', 1.1)
        self.brightness = rospy.get_param('~brightness', 5)
        
        # 获取输入和输出话题名称
        input_topic = rospy.get_param('~input_topic', '/camera/rgb/image_raw')
        output_topic = rospy.get_param('~output_topic', '/camera/rgb/image_filtered')
        
        # 创建发布者和订阅者
        self.image_pub = rospy.Publisher(output_topic, Image, queue_size=10)
        self.image_sub = rospy.Subscriber(input_topic, Image, self.image_callback, queue_size=1)
        
        rospy.loginfo(f"Image filter node started. Subscribing to {input_topic}, publishing to {output_topic}")
        
    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 应用颜色校正滤镜
            filtered_image = self.apply_filter(cv_image)
            
            # 将处理后的图像发布到新话题
            filtered_msg = self.bridge.cv2_to_imgmsg(filtered_image, "bgr8")
            filtered_msg.header = data.header  # 保持原始消息的头部信息
            self.image_pub.publish(filtered_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
    
    def apply_filter(self, image):
        """应用颜色校正滤镜修正AstraPro摄像头的黄绿偏色"""
        # 创建查找表(LUT)进行颜色校正
        lut = np.zeros((256, 1, 3), dtype=np.uint8)
        for i in range(256):
            # B通道增强
            b_value = np.clip(int(i * self.b_gain), 0, 255)
            # G通道减弱
            g_value = np.clip(int(i * self.g_gain), 0, 255)
            # R通道轻微调整
            r_value = np.clip(int(i * self.r_gain), 0, 255)
            lut[i, 0, 0] = b_value
            lut[i, 0, 1] = g_value
            lut[i, 0, 2] = r_value
        
        # 应用颜色校正LUT
        corrected_image = cv2.LUT(image, lut)
        
        # 调整对比度和亮度
        corrected_image = cv2.convertScaleAbs(corrected_image, 
                                              alpha=self.contrast, 
                                              beta=self.brightness)
        
        return corrected_image

if __name__ == '__main__':
    try:
        node = ImageFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass