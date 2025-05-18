#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler
import math
import time

class CarryAndMoveManager:
    def __init__(self):
        rospy.init_node('carry_and_move_manager')
        
        # 状态变量
        self.object_grasped = False
        self.move_distance = 3.0  # 默认移动3米
        
        # 发布者，用于触发物体检测和抓取
        self.carry_start_pub = rospy.Publisher('/spark_carry/start', Bool, queue_size=1)
        
        # 创建速度命令发布者（作为备用）
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 订阅抓取完成的消息
        rospy.Subscriber('/spark_carry/grasp_completed', Bool, self.grasp_callback)
        
        # 默认使用move_base，但在出错时会退回到简单命令
        self.use_move_base = True
        
        rospy.loginfo("初始化完成，等待抓取和移动命令")

    def wait_for_transform_available(self, timeout=20.0):
        """等待TF转换可用"""
        import tf
        tf_listener = tf.TransformListener()
        start_time = rospy.Time.now()
        r = rospy.Rate(2)  # 2Hz
        
        rospy.loginfo("等待TF转换可用...")
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            try:
                # 检查TF是否可用
                tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                rospy.loginfo("TF转换已可用")
                return True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF转换尚不可用，继续等待...")
                r.sleep()
        
        rospy.logwarn("等待TF转换超时，将使用简单的速度命令")
        return False

    def grasp_callback(self, msg):
        self.object_grasped = msg.data
        if self.object_grasped:
            rospy.loginfo("物体已抓取，准备移动")
            # 等待1秒确保抓取稳定
            rospy.sleep(1.0)
            
            # 检查TF是否可用，决定使用哪种移动方式
            self.use_move_base = self.wait_for_transform_available()
            
            if self.use_move_base:
                # 初始化move_base客户端
                self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                wait_result = self.move_base_client.wait_for_server(rospy.Duration(5.0))
                
                if wait_result:
                    rospy.loginfo("已连接到move_base服务器，将使用导航避障")
                else:
                    rospy.logwarn("无法连接到move_base服务器，将使用简单的前进命令")
                    self.use_move_base = False
            
            # 开始移动
            self.move_forward(self.move_distance)
    
    def move_forward(self, distance):
        """向前移动指定距离（单位：米）"""
        rospy.loginfo(f"开始向前移动 {distance} 米")
        
        if self.use_move_base:
            # 使用move_base进行导航（带避障）
            self.navigate_forward(distance)
        else:
            # 使用简单的速度命令前进
            self.simple_move_forward(distance)
    
    def navigate_forward(self, distance):
        """使用move_base导航向前移动（带避障）"""
        # 获取当前位置作为起点
        # 注意：实际应用中应该从tf获取当前位置
        # 这里简化为从原点出发，向x轴正方向移动
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标位置：在base_link坐标系中向前移动distance米
        goal.target_pose.pose.position.x = distance
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0.0
        
        # 保持当前朝向
        q = quaternion_from_euler(0, 0, 0)  # 朝向不变
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        rospy.loginfo("使用move_base导航前进...")
        self.move_base_client.send_goal(goal, done_cb=self.navigation_done_callback)
    
    def navigation_done_callback(self, status, result):
        """导航完成的回调函数"""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("导航成功完成！")
        else:
            rospy.logwarn(f"导航未成功完成，状态码: {status}")
    
    def simple_move_forward(self, distance):
        """使用简单的速度命令前进（无避障）"""
        speed = 0.2  # 速度，单位：米/秒
        duration = distance / speed  # 移动时间
        
        # 创建速度命令
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        # 记录开始时间
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo(f"使用速度命令前进，预计时间: {duration} 秒")
        
        # 循环发送速度命令直到达到指定时间
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止移动
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("移动完成！")
    
    def start_task(self):
        """开始执行任务：抓取物体然后移动"""
        rospy.loginfo("开始新任务：抓取物体并前进")
        
        # 发送开始抓取信号
        self.carry_start_pub.publish(True)
        return True
    
    def run(self):
        """主循环"""
        rospy.loginfo("等待命令...")
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = CarryAndMoveManager()
        # 自动开始任务
        rospy.sleep(2.0)  # 等待2秒让系统稳定
        manager.start_task()
        manager.run()
    except rospy.ROSInterruptException:
        pass