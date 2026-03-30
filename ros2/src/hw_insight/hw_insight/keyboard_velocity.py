#!/usr/bin/env python3

#import keyboard
import math
import sys
import tty
import termios
import select
import time
import rclpy
from rclpy.node import Node
from hw_interface.msg import HWSimpleKeyboardInfo

# NED坐标系每个方向上的速度变化量和最大值
X_STEP = 5
X_MAX = 100
Y_STEP = 5
Y_MAX = 100
Z_STEP = 2
Z_MAX = 50
YAW_STEP = 2
YAW_MAX = 300

# 阻塞方式读取按键 ##############################################################
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# 非阻塞方式读取按键 ##############################################################
def getch_with_timeout(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            return ch
        else:
            #print(f"{timeout}秒内没有获取到按键")
            return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# 读取键盘信息，并发布速度控制信息 ##############################################################
class KeyboardVelocity(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_velocity')

        # 初始速度为0
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        # 初始为不追踪模式
        self.track = 0
        self.init = True

        # 将速度信息发布到 /hw_insight/keyboard_velocity，接口类型是 HWSimpleKeyboardInfo
        self.pub = self.create_publisher(HWSimpleKeyboardInfo, "/hw_insight/keyboard_velocity", 1)

        # 发布信息的定时任务
        self.timer1 = self.create_timer(0.2, self.timer_callback_pub)

        # 读取键盘的定时任务
        self.timer2 = self.create_timer(0.1, self.timer_callback_key)


    # 各个方向速度归零逻辑，该方向无按键的情况下，将速度逐步归零
    def velocity_to_zero(self, x: bool, y: bool, z: bool, yaw: bool):
        if x:
            self.x = self.x - X_STEP if self.x > 0 else self.x + X_STEP if self.x < 0 else self.x
        if y:
            self.y = self.y - Y_STEP if self.y > 0 else self.y + Y_STEP if self.y < 0 else self.y
        if z:
            self.z = self.z - Z_STEP if self.z > 0 else self.z + Z_STEP if self.z < 0 else self.z
        if yaw:
            self.yaw = self.yaw - YAW_STEP if self.yaw > 0 else self.yaw + YAW_STEP if self.yaw < 0 else self.yaw


    # 读取键盘信息，并设置速度值
    def timer_callback_key(self):
        key = getch_with_timeout(0.5)
        # 处理按键值
        if key == 'w' or key == 'W':
            self.x = min(self.x + X_STEP, X_MAX)
            self.velocity_to_zero(False, True, True, True)
            print("前进")
        elif key == 's' or key == 'S':
            self.x = max(self.x - X_STEP, -X_MAX)
            self.velocity_to_zero(False, True, True, True)
            print("后退")
        elif key == 'a' or key == 'A':
            self.y = max(self.y - Y_STEP, -Y_MAX)
            self.velocity_to_zero(True, False, True, True)
            print("左移")
        elif key == 'd' or key == 'D':
            self.y = min(self.y + Y_STEP, Y_MAX)
            self.velocity_to_zero(True, False, True, True)
            print("右移")
        elif key == 'i' or key == 'I':
            self.z = max(self.z - Z_STEP, -Z_MAX)
            self.velocity_to_zero(True, True, False, True)
            print("上升")
        elif key == 'k' or key == 'K':
            self.z = min(self.z + Z_STEP, Z_MAX)
            self.velocity_to_zero(True, True, False, True)
            print("下降")
        elif key == 'j' or key == 'J':
            self.yaw = max(self.yaw - YAW_STEP, -YAW_MAX)
            self.velocity_to_zero(True, True, True, False)
            print("左转")
        elif key == 'l' or key == 'L':
            self.yaw = min(self.yaw + YAW_STEP, YAW_MAX)
            self.velocity_to_zero(True, True, True, False)
            print("右转")
        elif key == 'h' or key == 'H':
            self.x = 0
            self.y = 0
            self.z = 0
            self.yaw = 0
            print("停止")
        elif key == 't' or key == 'T':
            self.track = 1 - self.track
            if self.track == 1:
                print("自动追踪")
            else:
                print("取消追踪")
        else:
            # 退出按键
            if key is not None:
                key_ascii = ord(key)
                if key_ascii == 27 or key_ascii == 3:  # ESC键, Ctrl+C
                    exit(0)
            else:
            # 其他按键或者没有按键，将各个方向速度慢慢归零
                self.velocity_to_zero(True, True, True, True)


    # 发布速度信息
    def timer_callback_pub(self):
        msg = HWSimpleKeyboardInfo()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        msg.yaw = float(self.yaw)/100
        msg.track = self.track
        self.pub.publish(msg)

def main(args=None) -> None:
    print('Start keyboard velocity control node...')
    rclpy.init(args=args)
    keyboard_velocity = KeyboardVelocity()
    rclpy.spin(keyboard_velocity)
    keyboard_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
