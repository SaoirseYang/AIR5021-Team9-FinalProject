#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse

class ObjectTemplateRecognizer:
    def __init__(self):
        rospy.init_node('object_template_recognizer')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 创建订阅者，订阅相机图像
        self.image_sub = rospy.Subscriber("/camera/rgb/image_filtered", Image, self.image_callback)
        
        # 创建发布者，发布处理后的图像和检测结果
        self.image_pub = rospy.Publisher("/camera/rgb/image_detected", Image, queue_size=1)
        self.detection_pub = rospy.Publisher("/object_detection/result", Bool, queue_size=1)
        
        # 创建服务器，用于标定新的模板
        self.calibration_srv = rospy.Service('/object_detection/calibrate', Trigger, self.calibrate_callback)
        
        # 模板存储
        self.template = None
        self.template_features = None
        self.detector = cv2.SIFT_create()  # 使用SIFT特征检测器
        self.matcher = cv2.BFMatcher()  # 使用暴力匹配器
        
        # 参数
        self.match_threshold = rospy.get_param('~match_threshold', 0.7)
        self.min_good_matches = rospy.get_param('~min_good_matches', 10)
        
        # 标定模式标志
        self.calibrating = False
        self.calibration_countdown = 0
        
        rospy.loginfo("Object template recognizer initialized")
    
    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 处理校准
            if self.calibrating:
                if self.calibration_countdown > 0:
                    self.calibration_countdown -= 1
                    # 在图像上显示倒计时
                    cv2.putText(current_frame, f"Calibrating in {self.calibration_countdown//10 + 1}...", 
                                (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    # 获取模板
                    self.template = current_frame.copy()
                    # 提取特征
                    gray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
                    self.keypoints, self.template_features = self.detector.detectAndCompute(gray, None)
                    # 保存模板特征
                    rospy.loginfo("Template calibrated")
                    # 退出校准模式
                    self.calibrating = False
                    
                    # 在屏幕上显示确认消息
                    cv2.putText(current_frame, "Template calibrated!", 
                                (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 检测物体
            detection_result = False
            if self.template is not None and self.template_features is not None:
                # 灰度处理
                gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
                
                # 检测特征
                kp, desc = self.detector.detectAndCompute(gray, None)
                
                if desc is not None and len(desc) > 0:
                    # 进行特征匹配
                    matches = self.matcher.knnMatch(self.template_features, desc, k=2)
                    
                    # 应用比率测试
                    good_matches = []
                    for m, n in matches:
                        if m.distance < self.match_threshold * n.distance:
                            good_matches.append(m)
                    
                    # 如果有足够的好匹配，认为检测到目标
                    if len(good_matches) > self.min_good_matches:
                        detection_result = True
                        # 在图像上绘制结果
                        cv2.putText(current_frame, "Object Detected!", 
                                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        # 未检测到目标
                        cv2.putText(current_frame, "No Object Detected", 
                                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 发布处理后的图像
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(current_frame, "bgr8"))
            
            # 发布检测结果
            self.detection_pub.publish(Bool(detection_result))
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def calibrate_callback(self, req):
        """处理校准服务请求"""
        self.calibrating = True
        self.calibration_countdown = 30  # 3秒倒计时(假设10Hz)
        return TriggerResponse(
            success=True,
            message="Calibration started. Please place the object in front of the camera."
        )

if __name__ == '__main__':
    try:
        recognizer = ObjectTemplateRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass