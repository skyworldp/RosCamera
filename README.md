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
- 如果 `frame_rate` 未按期望生效，请：
  - 降低 `exposure_time`（曝光时间通常限制最高帧率），例如 `ros2 param set /hik_camera_node exposure_time 5000.0`。
  - 切换到 `Mono8` 或降低分辨率以减小每帧数据量。
  - 对 GigE 相机可尝试调整网卡 MTU（jumbo frames）和 packet size。

- 查看 topic 是否有发布：

```bash
ros2 topic list
ros2 topic info /cameraraw
```

- 使用图像查看工具：
  - rqt_image_view：`ros2 run rqt_image_view rqt_image_view`，选择 `/cameraraw`

