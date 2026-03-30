#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from hw_interface.msg import HWSimpleKeyboardInfo
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped
import numpy as np
import math


class Track(Node):
    def __init__(self) -> None:
        super().__init__('track')

        # 追踪逻辑主要数据 =====================================================================
        # 经过测试，跟踪的目标人物的速度为6.0米/秒，速度不会变化。以此前提来制定阶梯变速追踪方案
        # 从0到15米，每一米的距离给给出对应的速度，因此是15级阶梯
        # STEP_DISTANCE 没有实际作用，只是为了让下面的 STEP_SPEED 看起来更方便
        # STEP_SCALE 是计算得到的每一米内部的速度变化率
        self.STEP_LEVEL = 15
        self.STEP_DISTANCE =  [ 0.0,  1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,  8.0,  9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0]
        self.STEP_SPEED =     [-1.0, -0.5, 0.0, 2.0, 5.0, 6.0, 6.0, 6.0,  8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 18.0, 20.0]
        self.STEP_SCALE = [0.0] * 16
        for i in range(self.STEP_LEVEL):
            self.STEP_SCALE[i] = (self.STEP_SPEED[i+1] - self.STEP_SPEED[i])
        self.STEP_SCALE[self.STEP_LEVEL] = 0.0

        # 探测逻辑主要数据 =====================================================================
        # 当丢失目标超过一段时间后，会触发探测逻辑，无人机会飞到最后一次目标人物的坐标点后，在一定高度进行原地旋转弹出。
        # 初始化处于就处于目标丢失状态，无人机会在当前坐标旋转
        self.DETECT_YAW_SPEED = 2.0     # 旋转的速度，单位是弧度
        self.DETECT_Z_HEIGHT = -5.0     # 旋转的高度，单位米
        self.LOST_THRESHOLD = 200       # 丢失目标的时间阈值
        self.target_lost_count = self.LOST_THRESHOLD+1  # 丢失目标的计数，初始处于丢失目标状态


        # 和PX4通讯的QOS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 追踪参数，无人机和目标之间保持的距离。包括xy平面上的距离，以及z轴上的距离
        self.declare_parameter('keep_xy_distance', 3.0) # 目前没有使用，用的是 STEP_DISTANCE[2] 做为保持距离, 也就是速度为0时对应的距离
        self.declare_parameter('keep_z_distance', 3.5)  # 由于人物的识别点取的是脚底，因此这个距离要考虑到人的身高，比如无人机距离人物头顶1.5米，身高不超过2米，则距离3.5米

        # 切换模式用，包括起飞，着陆，返回，offboard等
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # 发布offboard控制模式，设置为速度控制模式
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # 发布速轨迹点，可控制位置，速度，加速度，航向角
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # 订阅无人机状态信息
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        # 订阅无人机当前位置
        self.sub_vehicle_pos = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position', self.vehicle_position_callback, qos_profile)
        # 订阅YOLO人员识别信息
        self.yolo_person_subscriber = self.create_subscription(PointStamped, '/yolo/person_position', self.yolo_person_callback, 1)
        # 订阅向下的距离传感器，用来降落
        self.distance_sensor_subscriber = self.create_subscription(Range, '/airsim_node/PX4/distance/DistanceDown', self.distance_sensor_callback, 1)
        # 订阅键盘控制信息
        self.hw_position_subscriber = self.create_subscription(HWSimpleKeyboardInfo, '/hw_insight/keyboard_velocity', self.hw_velocity_callback, 1)

        # 定义各种变量
        self.vehicle_status = VehicleStatus()               # 无人机状态信息
        self.distance_sensor = Range()                      # 无人机底部传感器信息
        self.hw_keyboard_control = HWSimpleKeyboardInfo()   # 键盘控制信息
        self.vehicle_position = VehicleLocalPosition()      # 无人机当前位置

        # 目标人物的坐标
        self.target_person_x = float('nan')
        self.target_person_y = float('nan')
        self.target_person_z = float('nan')

        # 定时器处理函数
        self.timer = self.create_timer(0.01, self.timer_callback)



# =============================================================================================================================
# ================= 各种订阅的回调函数，获取各种信息 ==========================================================================
# =============================================================================================================================
    # 获取无人机状态
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    # 获取向下的距离传感器信息
    def distance_sensor_callback(self, distance_sensor):
        self.distance_sensor = distance_sensor

    # 获取键盘控制信息
    def hw_velocity_callback(self, hw_keyboard_control):
        self.hw_keyboard_control = hw_keyboard_control
        #self.get_logger().info(f"键盘控制信息: {hw_keyboard_control}")



# =============================================================================================================================
# ================= 无人机状态切换的命令 ==========================================================================
# =============================================================================================================================
    # 解锁命令
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    # 加锁命令
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    # 切换到offboard模式
    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    # 降落命令
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # 返回命令
    def return_home(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    # 发布命令
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)



# =============================================================================================================================
# ================= 无人控制命令，位置控制和速度控制 ==========================================================================
# =============================================================================================================================
    # 设置为offboard模式，速度控制或者位置控制
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        if self.hw_keyboard_control.track == 1:
            msg.position = True
            msg.velocity = True
        else:
            msg.position = False
            msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # 设置位置
    def publish_velocity_setpoint_position(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # 设置速度
    def publish_velocity_setpoint_speed(self, x: float, y: float, z: float, yawspeed: float):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # 追踪设置，x,y是速度，z是高度，yaw是角度
    def publish_track(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), z]
        msg.velocity = [x, y, float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # 探测设置，x,y,z都是位置，yawspeed是角速度
    def publish_detect(self, x: float, y: float, z: float, yawspeed: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


# =============================================================================================================================
# ================= 自动追踪的关键信息和处理函数 ==========================================================================
# =============================================================================================================================
    # 订阅无人机位置信息的回调函数，入参是无人机位置信息
    def vehicle_position_callback(self, vehicle_position):
        # 将位置信息记录到变量中以便后续使用
        self.vehicle_position = vehicle_position
        # 如果从来没有获取到目标人物的位置，则将无人机当前的位置作为目标人物的位置，让目标丢失的探测逻辑得以正确运行
        if np.isnan(self.target_person_x):
            self.target_person_x = vehicle_position.x
            self.target_person_y = vehicle_position.y
        #self.get_logger().info(f"无人机当前位置: {vehicle_position}")


    # 订阅YOLO发布的目标人物位置信息，入参是人物位置信息
    def yolo_person_callback(self, person_position):
        # 排除无效位置信息
        if (person_position.point.x < -1000) or (person_position.point.x > 1000) :
            return
        # 记录目标人物位置信息
        self.target_person_x = person_position.point.x
        self.target_person_y = person_position.point.y
        self.target_person_z = person_position.point.z
        # 清空人物丢失计数器
        self.target_lost_count = 0

    # 阶梯方式设置平滑偏航角
    def do_smooth_yaw(self, target_yaw: float, current_yaw: float):
        # return target_yaw
        if abs(target_yaw - current_yaw) > 1.0 : # 相差大于120°时，直接设置为目标偏航角度
            smooth_yaw = target_yaw
        elif abs(target_yaw - current_yaw) > 0.5 : # 相差小于120°大于60°时，设置为差值的一半，降低转速
            smooth_yaw = current_yaw + (target_yaw - current_yaw)/2.0
        else : # 相差小于60°时，设置为差值的4分之1，更低转速
            smooth_yaw = current_yaw + (target_yaw - current_yaw)/4.0

        return smooth_yaw

    # 阶梯方式设置平滑xy的速度值
    def do_smooth_xy_speed(self, target_x: float, target_y: float, current_x: float, current_y: float):
        # 计算无人机坐标到目标人物坐标的距离
        distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        # 连接无人机坐标点和目标人物坐标点，并从两个坐标引出平行于X轴和Y轴的线，可以形成一个直角三角形
        # 计算X轴上的距离，以及Y轴上的距离和总距离的比例。也就是直角三角形的COS和SIN值
        scale_cos = (target_x - current_x) / distance   # target_x - current_x 已经包含了方向。Y轴同理
        scale_sin = (target_y - current_y) / distance

        # 取出距离的整数部分和小数部分，用来计算阶梯速度
        integer_part = int(distance)
        decimal_part = distance - integer_part
        if integer_part <= self.STEP_LEVEL :
            speed = self.STEP_SPEED[integer_part] + self.STEP_SCALE[integer_part] * decimal_part
        else :
            speed = self.STEP_SPEED[self.STEP_LEVEL]

        # 计算分量速度
        smooth_x = speed * scale_cos
        smooth_y = speed * scale_sin
        #print(f"distance={distance:.2f}, x={smooth_x:.2f}, y={smooth_y:.2f}")
        return smooth_x, smooth_y


    # 自动追踪目标
    def track_target(self):
        # 一段时间内没有获取到目标人物位置，则飞到最后一次检测到人物的位置，并上升到一定高度，原地旋转进行探测。
        self.target_lost_count = self.target_lost_count + 1
        if self.target_lost_count > self.LOST_THRESHOLD :
            self.publish_detect(self.target_person_x, self.target_person_y, self.DETECT_Z_HEIGHT, self.DETECT_YAW_SPEED)
            return

        # 获取到了目标人物位置，追踪
        # 记录无人机位置信息
        vehicle_x   = self.vehicle_position.x
        vehicle_y   = self.vehicle_position.y
        vehicle_z   = self.vehicle_position.z
        vehicle_yaw = self.vehicle_position.heading
        # 记录目标位置信息
        target_x = self.target_person_x
        target_y = self.target_person_y
        target_z = self.target_person_z

        # 计算无人机到目标的偏航角信息
        target_yaw = np.arctan2(target_y - vehicle_y, target_x - vehicle_x)
        # 偏航角平滑处理
        # smooth_yaw = self.do_smooth_yaw(target_yaw, vehicle_yaw)

        # 计算XY平面的目标点，并平滑处理
        smooth_x,smooth_y = self.do_smooth_xy_speed(target_x, target_y, vehicle_x, vehicle_y)

        # 将目标点设置到无人机
        self.keep_z_distance = self.get_parameter('keep_z_distance').value
        self.publish_track(smooth_x, smooth_y, target_z-self.keep_z_distance, target_yaw)
        # self.publish_velocity_setpoint_position(  # 发位置指令
        #     target_x, target_y, target_z - self.keep_z_distance, target_yaw
        # )
        # 计算距离
        distance = np.sqrt((target_x - vehicle_x) ** 2 + (target_y - vehicle_y) ** 2)
        #计算偏航角误差
        yaw_error = abs(target_yaw - vehicle_yaw)
        yaw_error = min(abs(yaw_error), 2 * math.pi - abs(yaw_error))
        # 添加数据记录（用于绘图）
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 期望速度（你的阶梯速度计算结果）
        smooth_x, smooth_y = self.do_smooth_xy_speed(target_x, target_y, vehicle_x, vehicle_y)
        desired_speed = np.sqrt(smooth_x ** 2 + smooth_y ** 2)

        # 实际速度
        actual_speed = np.sqrt(self.vehicle_position.vx ** 2 + self.vehicle_position.vy ** 2)

        # 保存到CSV
        current_time = self.get_clock().now().nanoseconds / 1e9

        # if not hasattr(self, 'csv_created'):
        #     with open('/tmp/tracking_data.csv', 'w') as f:
        #         f.write("time,distance,yaw_error,desired_speed,actual_speed\n")
        #     self.csv_created = True
        #     #self.get_logger().info(f"CSV文件已创建，路径: /tmp/tracking_data.csv")
        #
        # #self.get_logger().info(f"写入数据: time={current_time:.3f}, distance={distance:.3f}")
        # write_str = f"{current_time:.6f},{distance:.3f},{yaw_error:.3f},{desired_speed:.3f},{actual_speed:.3f}\n"
        # self.get_logger().info(f"写入字符串: {write_str.strip()}")
        # with open('/tmp/tracking_data.csv', 'a') as f:
        #     f.write(f"{current_time:.6f},{distance:.3f},{yaw_error:.3f},{desired_speed:.3f},{actual_speed:.3f}\n")

# =============================================================================================================================
# ================= 定时处理函数，主循环 ==========================================================================
# =============================================================================================================================
    # 主逻辑，根据键盘信息控制无人机
    def timer_callback(self):
        # offboard心跳
        self.publish_offboard_control_heartbeat_signal()
        #self.get_logger().info(f"nav_state = {self.vehicle_status.nav_state}, arm = {self.vehicle_status.arming_state}, velocity={[self.hw_keyboard_control.x, self.hw_keyboard_control.y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw]}")

        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            # 未解锁状态，收到上升命令，则解锁并起飞，切换到OFFBOARD状态
            if self.hw_keyboard_control.z < -5:
                self.engage_offboard_mode()
                self.arm()
                self.publish_velocity_setpoint_speed(self.hw_keyboard_control.x, self.hw_keyboard_control.y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw)
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # 已经解锁，且在OFFBOARD状态，则根据订阅/hw_insight/keyboard_velocity得到的值来控制无人机
            if self.hw_keyboard_control.track == 1:
                # 自动追踪模式
                #self.publish_track(-6.0, 0.0, -3.0, 3.1415926)
                self.track_target()
            elif self.hw_keyboard_control.z > 5 and self.distance_sensor.range < 5 :
                # 降低到一定高度，出发降落指令
                self.land()
            else:
                # 按照命令飞行
                # 提取无人机坐标的正前方在NED坐标轴上的分量
                fowrward_x = self.hw_keyboard_control.x * math.cos(self.vehicle_position.heading)
                fowrward_y = self.hw_keyboard_control.x * math.sin(self.vehicle_position.heading)
                # 提取无人机坐标的正左方在NED坐标轴上的分量
                left_x = -self.hw_keyboard_control.y * math.sin(self.vehicle_position.heading)
                left_y = self.hw_keyboard_control.y * math.cos(self.vehicle_position.heading)
                #ydx = np.tan(yaw)
                self.publish_velocity_setpoint_speed(fowrward_x+left_x, fowrward_y+left_y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw)
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # 降落状态下，如果收到上升指令，则结束降落状态，切换到OFFBOARD，并重新飞行
            if self.hw_keyboard_control.z < -5:
                self.engage_offboard_mode()
                self.publish_velocity_setpoint_speed(self.hw_keyboard_control.x, self.hw_keyboard_control.y, self.hw_keyboard_control.z, self.hw_keyboard_control.yaw)
        else:
            # 其他状态，正常逻辑不应该会到该状态，报错并返回
            self.land()
            #self.return_home()



def main(args=None) -> None:
    print('Starting move velocity node...')
    rclpy.init(args=args)
    track = Track()
    rclpy.spin(track)
    track.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)