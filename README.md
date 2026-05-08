# ROS2+PX4+AirSim 仿真环境搭建与使用教程

## 资源包说明

本教程所需的所有资源已打包为「AirSim相关资源包」，包含以下内容：
- AirSim（已打包为exe文件）
- PX4
- DDS-Agent
- ROS2

**下载链接**：
- 夸克网盘：`https://pan.quark.cn/s/421688f767a3`
- 提取码：无需提取码，打开夸克APP即可获取

## 目录

1. [环境准备](#环境准备)
2. [PX4 安装与配置](#px4-安装与配置)
3. [DDS-Agent安装](#dds-agent安装)
4. [ROS2安装](#ros2安装)
5. [AirSim 安装与配置](#airsim-安装与配置)
6. [系统启动与运行](#系统启动与运行)
7. [无人机控制方法](#无人机控制方法)
8. [通信验证](#通信验证)
9. [常见问题与解决方案](#常见问题与解决方案)

## 环境准备

### 硬件要求

- 推荐配置：CPU i5及以上，内存8GB及以上，显卡支持OpenGL 4.3及以上
- 操作系统：Windows 10/11 + WSL2 (Ubuntu 20.04或22.04)

### 软件要求

- WSL2 (Windows Subsystem for Linux 2)
- CMake 4.0及以上（如果编译AirSim报错，需要升级）
- Python 3.7及以上

## PX4 安装与配置

### 1. WSL配置

**重要注意事项**：为避免路径问题，建议在创建WSL用户时使用`hw`作为用户名，这样可以与PX4包中的路径信息保持一致。

如果已经使用了其他用户名，可以：

1. 重新安装WSL并创建`hw`用户（推荐）
2. 在现有WSL中添加`hw`用户并赋予sudo权限，然后切换到该用户继续安装
3. 如果坚持使用自己的用户名，进入PX4目录后需要先执行`make clean`和`make distclean`，但这会删除已下载的内容，速度较慢且可能出错

### 2. 安装依赖

在WSL中执行以下命令：

```bash
sudo apt update
sudo apt install -y pip libpulse-mainloop-glib0 zip
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip config set global.trusted-host mirrors.aliyun.com
echo 'export PATH=/home/hw/.local/bin:$PATH' >> ~/.bashrc
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
echo 'export DISPLAY=:0' >> ~/.bashrc
```

### 3. PX4安装

```bash
# 复制PX4安装包到WSL中
cp /mnt/f/install/px4v1.15.2.zip ~/

# 解压安装包
unzip px4v1.15.2.zip

# 进入PX4目录
cd px4v1.15.2

# 运行安装脚本
./Tools/setup/ubuntu.sh
```

### 4. PX4编译

```bash
# 编译PX4
make px4_sitl_default

# 编译完成后，设置环境变量
echo 'export PX4_SIM_HOST_ADDR=172.31.48.1' >> ~/.bashrc  # 注意：将IP地址改为自己Windows的IP地址
source ~/.bashrc

# 开放端口
sudo ufw allow 4560
sudo ufw allow 10049
```

## DDS-Agent安装

### 1. 安装步骤

```bash
# 将Micro-XRCE-DDS-Agent.zip拷贝到/home/hw目录
cp /path/to/Micro-XRCE-DDS-Agent.zip /home/hw/

# 解压缩
unzip Micro-XRCE-DDS-Agent.zip

# 进入build目录
cd Micro-XRCE-DDS-Agent/build

# 编译和安装
make
sudo make install
sudo ldconfig /usr/local/lib/

# 验证安装
MicroXRCEAgent --help
```

**重要注意事项**：建议使用与教程完全相同的目录结构，这样遇到的问题会少很多，而且遇到问题后也更容易得到支持。

## ROS2安装

### 1. 安装步骤

```bash
# 从「AirSim相关资源包」中复制ROS2安装包到/home/hw目录
cp /path/to/AirSim相关资源包/hw-ros2.zip /home/hw/

# 解压缩
unzip hw-ros2.zip

# 进入setup目录并运行安装脚本
cd hw-ros2/setup
./setup.sh

# 拷贝自己的ros2文件到工作空间（覆盖ros文件夹）
cp -r /path/to/AirSim相关资源包/ros2/* /home/hw/hw-ros2/ros2/

# 构建ROS2工作空间
cd /home/hw/hw-ros2/ros2
colcon build
```

## AirSim 安装与配置

### 1. 安装依赖

在Windows命令提示符中执行以下命令：

```bash
pip install ultralytics
pip install "numpy<2.0" "opencv-python<4.10"
```

### 2. 配置AirSim

1. **下载AirSim资源包**：从夸克网盘下载「AirSim相关资源包」
   - 链接：`https://pan.quark.cn/s/421688f767a3`
   - 提取码：无需提取码，打开夸克APP即可获取
2. 解压下载的资源包
3. 将`settings.json`拷贝到AirSim的配置目录
4. 运行`AirSim.exe`

## 系统启动与运行

### 1. 启动顺序

1. **启动AirSim**：在Windows中运行`AirSim.exe`
   <img width="933" height="380" alt="image" src="https://github.com/user-attachments/assets/2996881a-f37b-4d79-9610-a5d13026f60a" />

3. **启动PX4**：在WSL窗口中执行
   ```bash
   cd /home/hw/px4v1.15.2
   make px4_sitl_default none_iris
   ```
   <img width="831" height="444" alt="image" src="https://github.com/user-attachments/assets/a910b58a-5778-4335-ac47-3e7ae0d80f18" />
   显示ready takeoff表示连接成功
   <img width="641" height="194" alt="image" src="https://github.com/user-attachments/assets/0f9cbb3d-d2c6-42d8-a2f4-af272fbdfc6f" />

5. **启动DDS Agent**：在另一个WSL窗口中执行
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
   <img width="1096" height="281" alt="image" src="https://github.com/user-attachments/assets/56124485-7dab-4ebc-af8a-b344ede87205" />

6. **启动ROS2节点**：在另一个WSL窗口中执行
   ```bash
   cd /home/hw/hw-ros2/ros2
   source install/local_setup.sh
   ros2 launch hw_insight track.launch.py
   ```
   出现connected，以及initialized即代表成功
   
8. **启动键盘控制**：在另一个WSL窗口中执行
   ```bash
   cd /home/hw/hw-ros2/ros2
   source install/local_setup.sh
   ros2 run hw_insight keyboard_velocity
   ```

### 2. 验证启动状态

- AirSim窗口应该显示仿真环境
- PX4终端应该显示无人机状态信息
- DDS Agent终端应该显示连接成功信息
- ROS2终端应该显示节点启动成功信息
- 键盘控制窗口应该可以接收键盘输入

## 无人机控制方法

### 1. 键盘控制

在启动`ros2 run hw_insight keyboard_velocity`后，保持该窗口有焦点，即可用键盘控制无人机。

- **I**：控制上升
- **K**：控制下降
- **J**：控制左转
- **L**：控制右转
- **W**：控制前进
- **A**：控制左移
- **S**：控制后退
- **D**：控制右移

### 2. 手柄控制

场景中的人物需要使用手柄进行控制。具体操作方法如下：

1. 连接手柄到电脑
2. 在AirSim窗口中，手柄将自动被识别
3. 使用手柄的摇杆和按钮控制人物的移动和动作

### 3. ROS2命令控制

使用ROS2话题发布控制命令：

```bash
# 发布位置控制命令
ros2 topic pub /fmu/in/setpoint_position/local geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
  pose: {
    position: {
      x: 5.0,
      y: 0.0,
      z: 2.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0
    }
  }
}" -r 10
```

## 通信验证

### 1. 检查ROS2话题

```bash
# 查看所有ROS2话题
ros2 topic list

# 查看无人机状态话题
ros2 topic echo /fmu/out/vehicle_status
```

### 2. 检查DDS通信

```bash
# 查看DDS参与者
fastdds discovery -l
```

### 3. 检查AirSim连接

在AirSim窗口中，点击`Help` -> `About`，查看连接状态。

## 常见问题与解决方案

### 1. AirSim编译报错

**问题**：编译AirSim时出现错误
**解决方案**：升级CMake到4.0版本，可以到[CMake官网](https://cmake.org/download/)下载最新版本。

### 2. PX4与AirSim连接失败

**问题**：PX4无法连接到AirSim
**解决方案**：

- 确保Windows防火墙已关闭或开放了4560和10049端口
- 检查`PX4_SIM_HOST_ADDR`环境变量是否设置为正确的Windows IP地址
- 确保AirSim正在运行

### 3. ROS2话题无数据

**问题**：ROS2话题没有数据
**解决方案**：

- 检查DDS Agent是否正在运行
- 检查PX4与ROS2桥接是否成功
- 重新启动所有组件

### 4. 无人机无法起飞

**问题**：无人机无法起飞
**解决方案**：

- 检查AirSim是否正确加载了无人机模型
- 检查PX4参数设置是否正确
- 尝试使用键盘控制窗口的`I`键起飞

## 总结

本教程详细介绍了ROS2+PX4+AirSim仿真环境的搭建与使用方法，包括环境准备、PX4安装与配置、DDS-Agent安装、ROS2安装与配置、AirSim安装与配置、系统启动与运行、无人机控制方法、通信验证以及常见问题与解决方案。

### 特别说明

- **ROS2和DDS Agent**：已准备就绪，可以直接按照教程中的步骤进行安装和配置。
- **AirSim**：已打包为exe文件，直接运行即可。网盘链接将在后续提供。

按照本教程的步骤操作，具备基础Linux和ROS2知识的读者应该能够独立完成整个仿真环境的搭建与无人机控制操作。

**重要注意事项**：建议使用与教程完全相同的目录结构，这样遇到的问题会少很多，而且遇到问题后也更容易得到支持。

如果在操作过程中遇到问题，请参考常见问题与解决方案部分，或查阅相关官方文档。
