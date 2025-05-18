#!/bin/bash

# 颜色定义
Info="\033[32m[信息]\033[0m"
Error="\033[31m[错误]\033[0m"
Green_font_prefix="\033[32m"
Red_font_prefix="\033[31m"
Blue_font_prefix="\033[34m"
Font_color_suffix="\033[0m"

# 打印命令函数
print_command() {
    echo -e "\033[33m$1\033[0m"
}

# 获取ROS版本
ROSVER=`/usr/bin/rosversion -d`
PROJECTPATH=$(cd `dirname $0`; pwd)
source ${PROJECTPATH}/devel/setup.bash

# 设置相机类型
CAMERATYPE="astrapro"  # 默认相机类型，可以根据需要更改

# 显示信息
echo -e "${Info}" 
echo -e "${Info}启动简化版视觉抓取功能" 
echo -e "${Info}" 
echo -e "${Info}请确定："
echo -e "${Info}       A.摄像头已正确安装并连接。"
echo -e "${Info}       B.机械臂正常上电。" 
echo -e "${Info}       C.准备好可吸附的${Red_font_prefix}红色${Font_color_suffix}物品。" 
echo -e "${Info}退出请输入：Ctrl + c " 
echo -e "${Info}" 

echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

# 启动视觉抓取功能
echo -e "${Info}启动视觉抓取系统..." 
print_command "roslaunch spark_carry_object simple_carry_object.launch camera_type_tel:=${CAMERATYPE}"
roslaunch spark_carry_object simple_carry_object.launch camera_type_tel:=${CAMERATYPE}