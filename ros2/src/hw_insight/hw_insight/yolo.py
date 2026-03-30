#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from hw_interface.msg import HWSimpleKeyboardInfo
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
import message_filters
import time
from datetime import datetime

# 需要让RGB和DEPTH图像的时间戳完全对应才行

class YOLO(Node):
    def __init__(self) -> None:
        super().__init__('yolo')

        # 读取相机名字参数
        self.declare_parameter('camera_name', "CameraDepth1")
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # 订阅相机的RGB图片和深度图片。RGB图片用来给YOLO识别物体，深度图片用来根据YOLO识别的结果计算坐标值
        self.rgb_sub = message_filters.Subscriber(self, Image, f'/airsim_node/PX4/{self.camera_name}/Scene')
        self.depth_sub = message_filters.Subscriber(self, Image, f'/airsim_node/PX4/{self.camera_name}/DepthPlanar')
        # 创建一个近似时间同步器，确保RGB图片和DEPTH图片的同步
        # 参数：订阅者列表，队列大小，时间戳容差（秒）
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 2, 0.01)
        # 注册同步后的回调函数
        self.time_synchronizer.registerCallback(self.sync_callback)

        # 读取YOLO模型参数，选择合适的模型
        self.declare_parameter('yolo_model', "yolov8n.pt")
        self.yolo_model = self.get_parameter('yolo_model').get_parameter_value().string_value

        # 读取YOLO模型，CvBridge用来在RGB和ROS的image格式间进行转换
        pkg_share = get_package_share_directory('hw_insight')
        yolo_model_path = os.path.join(pkg_share, 'yolo/' + self.yolo_model)  # 可换成其他模型
        self.model = YOLO(yolo_model_path)
        self.bridge = CvBridge()

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.f = open(f'/tmp/yolo_data_{timestamp}.csv', 'w')
        self.f.write('time,has_detection\n')  # 表头

        # 用来发布YOLO识别后，带识别方框的RGB图片
        self.yolo_pub = self.create_publisher(Image, '/yolo/output', 1)
        # 用来发布识别到的人的位置信息，基于PX4_odom的坐标
        self.point_pub = self.create_publisher(PointStamped, '/yolo/person_position', 1)

        # TF2 buffer + listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Camera={self.camera_name}, YOLO={self.yolo_model}, Started.")

    # 订阅深度图信息的回调函数
    def depth_callback(self, msg):
        try:
            # 缓存最近的深度图信息
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    # 同步回调函数，它会同时接收到 rgb_msg 和 depth_msg
    def sync_callback(self, rgb_msg, depth_msg):
        # 记录接收时间（ROS时间）
        receive_time = self.get_clock().now()

        # 获取图片发布时间
        publish_time = rclpy.time.Time.from_msg(rgb_msg.header.stamp)

        # 计算传输延迟（从发布到接收）
        transport_delay = (receive_time - publish_time).nanoseconds / 1e6  # 转换为毫秒

        # 记录处理开始时间
        process_start = time.perf_counter()
        # 这个回调函数只在接收到时间戳相近的RGB和深度图时才被触发。
        try:
            # 将ROS图像消息转换为OpenCV图像
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # 使用YOLO模型进行目标检测, classes=[0] 表示只检测 "person" 类别, verbose=False 减少控制台输出
        # conf置信度0.6以上, 后续训练专用模型后可以适当降低，以更好的识别远处的目标人物，目前识别距离极限20米左右
        # results = self.model(img, classes=[0], conf=0.6, verbose=False)
        # 检测多种车辆类别（根据YOLOv8类别ID）
        # 2: car, 5: bus, 7: truck, 3: motorcycle, 4: airplane
        results = self.model(img, classes=[0], conf=0.6, verbose=False)

        # 记录：时间, 是否有目标(1或0)
        has_detection = 1 if len(results[0].boxes) > 0 else 0
        self.f.write(f'{time.time():.3f},{has_detection}\n')
        self.f.flush()

        # 可视化检测结果
        annotated_frame = results[0].plot()
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        out_msg.header = rgb_msg.header  # 保持header一致
        self.yolo_pub.publish(out_msg)

        # 计算YOLO处理时间
        yolo_process_time = (time.perf_counter() - process_start) * 1000  # 毫秒

        # 计算总端到端延迟
        current_time = self.get_clock().now()
        total_delay = (current_time - publish_time).nanoseconds / 1e6  # 毫秒

        # 打印延迟信息（每30帧打印一次，避免刷屏）
        if not hasattr(self, 'frame_count'):
            self.frame_count = 0
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"延迟统计 | 传输: {transport_delay:.1f}ms | "
                f"YOLO处理: {yolo_process_time:.1f}ms | "
                f"总端到端: {total_delay:.1f}ms"
            )

        # 测试YOLO运行时间
        # start_time = time.perf_counter()
        # YOLO模型运行一次
        # results = self.model(img, classes=[0], conf=0.7, verbose=False)
        # end_time = time.perf_counter()
        # execution_time = end_time - start_time
        # self.get_logger().info(f"TIME = {execution_time:.6f}")
        # 直接发布原始图片，检测延时
        # self.yolo_pub.publish(rgb_msg)
        # return

        # 如果检测到目标
        if len(results[0].boxes) > 0:
            # 只处理第一个检测到的目标
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # 计算目标中心点像素坐标
            u = (x1 + x2) // 2
            # v = y2  # 使用底部中心点
            # 修改后（车辆可以使用中心点或稍微偏下）：
            v = (y1 + y2) // 2  # 使用中心点

            # 检查像素坐标是否在图像范围内
            if not (0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]):
                self.get_logger().warn("Pixel coordinate out of depth image bounds.")
                return

            # 从同步好的深度图中获取深度值
            depth = depth_image[v, u]

            # 检查深度值是否有效
            if depth == 0 or np.isinf(depth) or np.isnan(depth):
                self.get_logger().warn(f"Invalid depth value at ({u},{v}): {depth}")
                return

            # self.get_logger().info(f"Detected person at pixel ({u}, {v}) with depth: {depth:.2f} meters")

            # 相机内参, 分辨率 800x600，水平视场角 FOV = 120° (后续考虑从 CameraInfo 话题获取，这里暂时硬编码)
            HFOV_deg = 120.0
            HFOV_rad = math.radians(HFOV_deg)
            W = 600.0
            H = 480.0
            # 根据公式 fx = W / (2 * tan(FOV/2)) 计算焦距（像素）
            fx = W / (2 * math.tan(HFOV_rad / 2))
            fy = fx  # AirSim像素是方形的
            cx = W / 2
            cy = H / 2

            # 像素坐标到相机坐标系转换
            x_cam = (u - cx) * depth / fx
            y_cam = (v - cy) * depth / fy
            z_cam = float(depth)

            # 创建 PointStamped 消息
            point_camera = PointStamped()
            point_camera.header = rgb_msg.header
            point_camera.header.frame_id = 'PX4/CameraDepth1_optical'  # 告诉TF这是相机坐标
            point_camera.point.x = x_cam
            point_camera.point.y = y_cam
            point_camera.point.z = z_cam

            try:
                # 将点从相机坐标系转换到PX4_odom坐标系
                target_frame = 'PX4_odom'
                point_odom = self.tf_buffer.transform(
                    point_camera,
                    target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.01)  # 设置一个合理的超时
                )

                # self.get_logger().info(f"Person position in {target_frame}: x={point_odom.point.x:.2f}, y={point_odom.point.y:.2f}, z={point_odom.point.z:.2f}")
                self.point_pub.publish(point_odom)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"TF transform failed: {e}")


def main(args=None) -> None:
    print('Starting YOLO test1 node...')
    rclpy.init(args=args)
    yolo = YOLO()
    rclpy.spin(yolo)
    yolo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)