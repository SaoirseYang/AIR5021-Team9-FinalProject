#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import sys

def start_task():
    rospy.init_node('start_carry_nav', anonymous=True)
    pub = rospy.Publisher('/spark_carry/start', Bool, queue_size=1)
    rospy.sleep(1.0)  # 等待发布者初始化
    
    # 发送开始信号
    msg = Bool()
    msg.data = True
    pub.publish(msg)
    rospy.loginfo("已发送开始信号")

if __name__ == '__main__':
    try:
        start_task()
    except rospy.ROSInterruptException:
        pass