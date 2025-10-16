# hik_camera (ROS2)

这是一个基于海康（Hikvision）相机 SDK 的 ROS2 相机节点示例工程（Humble）。

目录结构（相关）：
- src/hik_camera/src/hik_camera_node.cpp  -- ROS2 节点实现
- src/hik_camera/src/cameral_func/HikCamera.cpp -- 相机 SDK 封装
- launch/hik_camera_launch.py -- 启动文件（可选启动 RViz）

## 依赖
- ROS2 Humble（rclcpp, sensor_msgs 等）
- OpenCV / cv_bridge
- Hikvision SDK (`libMvCameraControl.so`) 放在 `src/hik_camera/include/hik_camera/lib/64`

## 构建
在工作区根目录运行：

```bash
# zsh
cd /home/skyworld/文档/Ros
colcon build --packages-select hik_camera --event-handlers console_direct+
```

构建完成后在每个新的终端 source 安装：

```bash
source install/setup.zsh
```

## 运行
使用 launch（包含可选的 rviz）:

```bash
source install/setup.zsh
ros2 launch hik_camera hik_camera_launch.py rviz:=false
```

或直接运行节点：

```bash
source install/setup.zsh
ros2 run hik_camera hik_camera_node
```
1:  LogType:Level3;  LogTime:2025-10-13 12:51:41:419 ;  LogContent:DevID:Virtual USB3 Vision  Description:[OpenInterfaceInter]Get cti version failed, nRet[0x8000000c];  LogSource:libMvCameraControl.so(GenTLManager.cpp-L1360);  LogProcessName:hik_camera_node;  LogProcessID:11534

日志会输出相机的 `camera reported fps` 以及当前帧的分辨率（例如 `camera reported fps: 20.37 Hz, frame size: 3072x2048`）。
若将 `pixel_format` 设置为 Bayer8（如 `PixelType_Gvsp_BayerRG8`），节点会自动使用 OpenCV 去马赛克并以 BGR8 发布，便于 RViz、rqt_image_view 等工具直接显示。

## 修改参数（运行时可改）
节点名称为 `/hik_camera_node`（launch 或 ros2 run 启动后可见），常用参数：

- `exposure_time` (double, 单位 microsecond)
- `gain` (double)
- `trigger_mode` (bool)
- `frame_rate` (double, Hz)
- `pixel_format` (int, 使用 SDK 常量，如 PixelType_Gvsp_Mono8、PixelType_Gvsp_BayerRG8 等)
- `frame_id` (string)

在单独终端（已 source 环境）使用 `ros2 param` 修改：

```bash
# 例如把曝光调成 10000 微秒
ros2 param set /hik_camera_node exposure_time 10000.0
# 设置期望帧率为 50 Hz
ros2 param set /hik_camera_node frame_rate 50.0
# 设置像素格式（示例值，这里使用 MONO8 对应的 SDK 值）
ros2 param set /hik_camera_node pixel_format 17301505
# 常见 Bayer8 对应值：
#   PixelType_Gvsp_BayerGR8 = 17301512
#   PixelType_Gvsp_BayerRG8 = 17301513
#   PixelType_Gvsp_BayerGB8 = 17301514
#   PixelType_Gvsp_BayerBG8 = 17301515
```


## 调试建议


## 运行示例：本地视频发布器 + 图像处理节点

下面是一个最小的运行说明，演示如何构建工作区、source 环境，并分别在两个终端启动视频发布节点（`hik_video_publisher_cpp`）和图像处理节点（`image_process`）。

注意：在每个新的终端中都需要 source 安装目录（或各包的 local_setup），否则 ros2 无法发现新构建的包。

1) 构建并 source（在工作区根目录执行）：

```bash
# zsh
cd /home/skyworld/文档/Ros
colcon build --event-handlers console_direct+
source install/setup.zsh
```

2) 在终端 A（启动本地视频发布器）：

```bash
# 在新的终端中先 source 工作区
source /home/skyworld/文档/Ros/install/setup.zsh
ros2 launch hik_video_publisher_cpp hik_video_publisher_launch.py rviz:=false
```

3) 在终端 B（启动图像处理节点）：

```bash
# 在新的终端中先 source 工作区
source /home/skyworld/文档/Ros/install/setup.zsh
ros2 launch image_process image_process_launch.py rviz:=false
```

可选：如果你希望直接运行编译出的可执行而不是 launch：

```bash
# 直接运行已安装的二进制（示例）
ros2 run hik_video_publisher_cpp hik_video_publisher
ros2 run image_process image_process_node
```

4) 常用调试命令（在第三个终端执行）：

```bash
# 列出节点
ros2 node list
# 检查话题
ros2 topic list
ros2 topic info /cameraraw
ros2 topic info /image_result
ros2 topic info /image_binary
```

日志或运行时常见问题：
- 如果报错 "package not found"，确认你在运行前已经 `source install/setup.zsh`，或者在包的 install 下手动 `source install/<pkg>/share/<pkg>/local_setup.zsh`。
- 如果 `image_process` 在调用模型时报错：确保模型文件路径正确且文件可读（默认硬编码路径在 `src/image_process/resources/`）。

```bash
ros2 topic list
ros2 topic info /cameraraw
```

- 使用图像查看工具：
  - rqt_image_view：`ros2 run rqt_image_view rqt_image_view`，选择 `/cameraraw`

