# HBUS 数据使用 mapping_lg.launch

本配置对应同一套设备的 `LIDAR_FRONT`、`CAM_FRONT_MID` 和 IMU，点云字段为
`x y z intensity timestampOffset`，均为 FLOAT32。`timestampOffset` 按微秒读取，
预处理乘以 0.001，写入算法使用的毫秒时间字段。

## 本次问题与修复

1. `mapping_lg.launch` 原先在 `laserMapping` 节点内再次加载了 `lg.yaml`，
   缺少相机内参。现在节点内加载 `camera_lg.yaml`，全局加载 `lg.yaml`。
   `lg.launch` 转为包含这个启动文件，避免两份配置不一致。
2. 原转换工具固定读取 `LIDAR_RIGHT`，却将 frame_id 写为 `lidar_front`。
   原 bag 首帧点云数据与 `LIDAR_RIGHT` 的 PCD 完全相同。用当前相机外参投影，
   34516 个点中没有点落入图像；真正的 `LIDAR_FRONT` 首帧有 29247 个点落入图像。
   新转换工具默认读取 `LIDAR_FRONT`，支持 `--lidar-dir` 显式选择传感器，
   并在转换报告中记录实际来源目录。
3. 前雷达与当前 IMU 的坐标轴不一致，单位外参旋转矩阵会造成后半段轨迹发散和
   跟踪丢失。依据这份 bag 的点云配准与 IMU 角速度，使用以下粗轴向修正：

   ```text
   p_imu = [z_lidar, y_lidar, -x_lidar] + extrinsic_T
   extrinsic_R = [ 0  0  1
                   0  1  0
                  -1  0  0 ]
   ```

   `Rcl/Pcl` 保持用户提供的相机标定值；`extrinsic_T` 也保留原值。
   这个旋转是依据当前数据验证的粗轴向修正，精确建图评估仍需要实测的雷达到 IMU
   安装外参。已开启重力对齐，将地图竖直方向对齐到 RViz 的 Z 轴。

## 当前数据

- 修正后的前雷达 bag：`/home/hyh/catkin_ws/hbus_ros1.bag`。
- 原右雷达 bag 备份：`/home/hyh/catkin_ws/hbus_ros1.lidar_right.bag`。
- 转换报告：`/home/hyh/catkin_ws/hbus_ros1.bag.json`。

修正后的 527 帧点云逐帧匹配 `LIDAR_FRONT` 的原始 PCD 数据；527 帧图像和
5202 条 IMU 消息与原 bag 序列化内容完全一致。

## 运行

终端一：

```bash
cd /home/hyh/catkin_ws
source devel/setup.bash
roslaunch fast_livo mapping_lg.launch
```

终端二：

```bash
source /home/hyh/catkin_ws/devel/setup.bash
rosbag play /home/hyh/catkin_ws/hbus_ros1.bag --clock
```

启动文件默认使用仿真时间，先启动节点再回放 bag。RViz 的 Fixed Frame 为
`camera_init`，图像订阅 `/rgb_img`，点云订阅 `/cloud_registered`。
实时传感器输入可用 `use_sim_time:=false`。

重新转换导出的数据时使用工作空间中的修正版工具：

```bash
/usr/bin/python3 /home/hyh/catkin_ws/src/FAST-LIVO2/scripts/hbus_to_rosbag.py \
  /home/hyh/hbus/hbus2rosbag \
  --lidar-dir LIDAR_FRONT --gyro-unit deg/s \
  -o /home/hyh/catkin_ws/hbus_front_new.bag
```

工具拒绝覆盖已有输出。若选择 `LIDAR_RIGHT`，需要使用右雷达对应的标定。
当前相机模型对应已经去畸变的 1920×1080 图像。

## 验证

传感器选择的两个 CLI 回归测试：

```bash
cd /home/hyh/catkin_ws/src/FAST-LIVO2
/usr/bin/python3 -m unittest discover -s scripts -p test_hbus_to_rosbag.py -v
```

完整回放检查 `/rgb_img` 的绿色、蓝色跟踪圆点，以及 `/cloud_registered` 的非空
点云，具体结果和截图保存到工作空间的 `diagnostics/hbus_lg` 目录。
