#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import tty
import termios
import time
from spark_carry_object.msg import position
from spark_carry_object.msg import status

class ManualArmControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("manual_arm_control", anonymous=True)
        
        # 创建发布者，增加调试输出
        print("正在初始化ROS发布者...")
        
        # 创建发布者，发布位置指令 - 同时尝试两种可能的话题名称
        self.pos_pub1 = rospy.Publisher("position_write_topic", position, queue_size=10)
        self.pos_pub2 = rospy.Publisher("/position_write_topic", position, queue_size=10)
        
        # 也尝试原型代码中可能使用的其他话题名称
        self.pos_pub3 = rospy.Publisher("position_write", position, queue_size=10)
        
        # 创建泵控制发布者
        self.pump_pub = rospy.Publisher("pump_topic", status, queue_size=1)
        
        # 初始化位置数据
        self.current_x = 160
        self.current_y = 0
        self.current_z = 55
        self.step = 5  # 每次移动的步长(mm)
        self.pump_status = 0  # 泵状态，0为关闭，1为开启
        
        # 等待连接建立（给ROS网络更多时间）
        print("等待ROS网络连接...")
        time.sleep(2)
        
        # 查看当前活动的话题
        print("正在检查可用的ROS话题...")
        try:
            # 这将打印所有活动的ROS话题，帮助诊断
            import subprocess
            result = subprocess.check_output(["rostopic", "list"])
            print("可用话题列表:")
            print(result.decode('utf-8'))
        except Exception as e:
            print("无法获取话题列表:", str(e))
        
        # 发布初始位置
        print("正在发布初始位置...")
        self.publish_position()
        
        print("=== 机械臂手动控制程序 ===")
        print("方向键控制:")
        print("  W/S: 控制Y轴 +/-")
        print("  A/D: 控制X轴 -/+")
        print("  Q/E: 控制Z轴 +/-")
        print("  P:   切换泵状态(开/关)")
        print("  +/-: 调整步长")
        print("  H:   回到初始位置(Home)")
        print("  R:   重新发送当前位置")
        print("  I:   显示调试信息")
        print("  X:   退出程序")
        print("当前位置: X={:.2f}, Y={:.2f}, Z={:.2f}, 步长={}mm, 泵状态={}".format(
            self.current_x, self.current_y, self.current_z, self.step, "开启" if self.pump_status else "关闭"))
    
    def publish_position(self):
        # 发布位置消息到所有可能的话题
        pos = position()
        pos.x = float(self.current_x)
        pos.y = float(self.current_y)
        pos.z = float(self.current_z)
        
        # 发布到所有可能的话题
        self.pos_pub1.publish(pos)
        self.pos_pub2.publish(pos)
        self.pos_pub3.publish(pos)
        
        print("发布位置: X={:.2f}, Y={:.2f}, Z={:.2f}, 步长={}mm, 泵状态={}".format(
            self.current_x, self.current_y, self.current_z, self.step, "开启" if self.pump_status else "关闭"))
        
        # 显示发布者信息，帮助诊断
        print("位置发布者1连接数: {}".format(self.pos_pub1.get_num_connections()))
        print("位置发布者2连接数: {}".format(self.pos_pub2.get_num_connections()))
        print("位置发布者3连接数: {}".format(self.pos_pub3.get_num_connections()))
    
    def toggle_pump(self):
        # 切换泵状态
        self.pump_status = 1 if self.pump_status == 0 else 0
        self.pump_pub.publish(self.pump_status)
        print("泵状态切换为: {}".format("开启" if self.pump_status else "关闭"))
    
    def get_key(self):
        # 获取键盘输入
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def home_position(self):
        # 回到初始位置
        self.current_x = 160
        self.current_y = 0
        self.current_z = 55
        self.publish_position()
        print("回到初始位置")
    
    def run(self):
        while not rospy.is_shutdown():
            key = self.get_key().lower()
            
            # 处理键盘输入
            if key == 'w':  # 增加Y
                self.current_y += self.step
            elif key == 's':  # 减少Y
                self.current_y -= self.step
            elif key == 'a':  # 减少X
                self.current_x -= self.step
            elif key == 'd':  # 增加X
                self.current_x += self.step
            elif key == 'q':  # 增加Z
                self.current_z += self.step
            elif key == 'e':  # 减少Z
                self.current_z -= self.step
            elif key == 'p':  # 切换泵状态
                self.toggle_pump()
            elif key == 'h':  # 回到初始位置
                self.home_position()
            elif key == 'r':  # 重新发送当前位置
                print("重新发送当前位置")
                self.publish_position()
            elif key == 'i':  # 显示调试信息
                print("\n=== 调试信息 ===")
                print("正在检查ROS话题连接状态...")
                try:
                    import subprocess
                    result = subprocess.check_output(["rostopic", "list"])
                    print("可用话题列表:")
                    print(result.decode('utf-8'))
                    
                    # 检查节点状态
                    result = subprocess.check_output(["rosnode", "list"])
                    print("可用节点列表:")
                    print(result.decode('utf-8'))
                except Exception as e:
                    print("无法获取ROS信息:", str(e))
            elif key == '+' or key == '=':  # 增加步长
                self.step += 1
                print("步长增加到{}mm".format(self.step))
            elif key == '-' or key == '_':  # 减少步长
                if self.step > 1:
                    self.step -= 1
                    print("步长减少到{}mm".format(self.step))
            elif key == 'x':  # 退出
                print("程序退出")
                break
            else:
                continue
            
            # 发布新位置
            if key in ['w', 's', 'a', 'd', 'q', 'e']:
                self.publish_position()
            
            # 防止发送过快
            rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        controller = ManualArmControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("发生错误:", str(e))