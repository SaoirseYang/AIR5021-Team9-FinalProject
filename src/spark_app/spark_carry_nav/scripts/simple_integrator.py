#!/usr/bin/env python3

import rospy
import math
import actionlib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SimpleIntegrator:
    def __init__(self):
        rospy.init_node('simple_integrator')
        
        # 状态变量
        self.object_grasped = False
        self.move_distance = 3.0  # 默认移动3米
        
        # 订阅抓取完成的消息
        rospy.Subscriber('/spark_carry/grasp_completed', Bool, self.grasp_callback)
        
        # 创建action client，用于监听导航任务的完成情况
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器连接...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base服务器已连接")
        
        # 创建发布者用于触发放下物体的动作
        self.release_pub = rospy.Publisher('/spark_carry/release_object', Bool, queue_size=1)
        
        rospy.loginfo("简单集成节点初始化完成")
    
    def grasp_callback(self, msg):
        self.object_grasped = msg.data
        if self.object_grasped:
            rospy.loginfo("物体已抓取，准备前进")
            # 等待1秒确保抓取稳定
            rospy.sleep(1.0)
            # 发送导航目标
            self.send_forward_goal(self.move_distance)
    
    def send_forward_goal(self, distance):
        """发送前进的导航目标并等待完成"""
        rospy.loginfo(f"发送前进 {distance} 米的导航目标")
        
        # 创建目标消息
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标位置：在base_link坐标系中向前移动distance米
        goal.target_pose.pose.position.x = distance
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0.0
        
        # 保持当前朝向
        goal.target_pose.pose.orientation.w = 1.0
        
        # 发送目标并设置完成后的回调函数
        self.move_base_client.send_goal(goal, done_cb=self.navigation_done_callback)
        rospy.loginfo("导航目标已发送，等待导航完成...")
    
    def navigation_done_callback(self, status, result):
        """导航完成后的回调函数"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功完成，准备放下物体")
            self.trigger_release_object()
        else:
            rospy.logwarn(f"导航未成功完成，状态码: {status}")
    
    def trigger_release_object(self):
        """触发放下物体的动作"""
        rospy.loginfo("发送放下物体指令")
        release_msg = Bool()
        release_msg.data = True
        self.release_pub.publish(release_msg)
        rospy.loginfo("放下物体指令已发送")
    
    def run(self):
        """主循环"""
        rospy.loginfo("等待抓取完成信号...")
        rospy.spin()

if __name__ == '__main__':
    try:
        integrator = SimpleIntegrator()
        integrator.run()
    except rospy.ROSInterruptException:
        pass