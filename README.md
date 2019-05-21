# TDPS-Project

UESTC TDPS Project Source

## Technical Route

### ROS System (Raspberry Pi 3B+) 

(/Raspi-ROS):

- Main system

(/Raspi\_AHRS):

- Razor IMU  

(/[TODO]):

- ydlidar  

(/Raspi-Cam):

- Traditional camera  

(/OpenMV)(NOT MY JOB!):

- OpenMV camera

### Arduino (Arduino UNO R3)

(/Arduino)

- L298P motor controller
- motor encoder

### Arduino (Arduino UNO R3) (NOT MY JOB!)

- PWM controller
- Robotic arm

## Work Report (工作报告)

### 寻线部分

#### 思路

公共部分：

- 首先将图像灰度化处理并缩小尺寸至树莓派的处理能力之内。
- 调用边缘检测过程，将路面图像的“粗糙程度”特征提取出来。
- 随后使用开运算增大图像的对比度。
- 将图像切分为上下两份，对两份图像分别做相同的处理。
- 对于分割后的每一份图像矩阵，对于每一列的像素进行求和操作，得到一个描述“粗糙程度”的特征沿水平方向上的分布。
- 对该数列减去它的平均数，并乘以单位阶跃函数。
- 将上一步得出的分布数组按索引加权积分，并除以该数列的平均数，得到一个坐标，该坐标即反映路面在此部分图像上的方向。

初始部分：

- 不断重复上述过程，获得最能拟合初始状态的分布函数及其参数，随后利用参数转入任务部分。

任务部分：

- 将上下两部分的方向作差并判断是否在中线的误差范围内，否则根据情况发出转向指令。
- 转向指令以偏差度数为度量，并分为转向和纠正横向偏移两个部分，前者是发送转向指令，后者是先发送一次转向指令，再发送一次相反方向和大小的转向指令。

#### 部分代码解析

##### 图像处理部分

```python
    ret, frame = cap.read() #从设备中读取一帧
    frame = cv2.resize(frame, (size_x, size_y)) # 缩小图片以降低运算量
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 灰度化处理
    src = cv2.Canny(grey, lowThreshold, lowThreshold * ratio, apertureSize=kernel_size) # 以大小为3的核函数进行边缘检测
    src_processed = cv2.morphologyEx(src, cv2.MORPH_CLOSE, dilate_kernel, iterations=1) # 以同样为3x3大小的核函数进行开运算
```

##### 方向计算部分

```python
    n_mean = np.mean(sum_array) # 计算数列的平均值
    if int(n_mean) is 0: # 全空的数列表示为检测到有效边缘，返回0
        return 0
    sum_array_fixed = sum_array
    sum_array_fixed[sum_array_fixed - n_mean < 0] = 0 # 等价于乘上单位阶跃函数
    left_side, right_side = find_side(sum_array_fixed) # 寻找有效的非0区间
    t_tot = 0
    for i_num in range(left_side, right_side):
        t_tot += i_num * sum_array_fixed[i_num] # 加权积分运算
    t_av = t_tot / (right_side - left_side) # 找出该分布的中心点索引
    n_mean_fixed = np.mean(sum_array_fixed) # 有效非0区间的平均数
    av = t_av / n_mean_fixed # 返回归一化的比例关系
    return int(av)
```

### 控制部分

#### ROS

ROS，是专为机器人软件开发所设计出来的一套电脑操作系统架构。它是一个开源的元级操作系统，提供类似于操作系统的服务，包括硬件抽象描述、底层驱动程序管理、共用功能的执行、程序间消息传递、程序发行包管理，它也提供一些工具和库用于获取、建立、编写和执行多机融合的程序。

#### 主线

Missions -> Goals -> AMCL -> Planner -> move_base -> cmd_vel/odom -> base_controller -> Motor

#### odom支线

编码器/9轴传感器 -> odom/imu -> odom_combined -> map

#### Missions： 任务调度器

控制启动相应的高层级的ROS launch文件。通过预先定义好的选项，在patio1和patio2间切换，并向下级节点下发任务列表，监控任务的每一个goals。是系统最先起始和最后结束的部分。

#### Goals： 目标调度器

每一个Goals是每个patio内的不同任务节点，同一个goal下任务的类型相同。实质上是ROS中的Service，由上一级的任务调度器启动，并将运行结果反馈以及接受执行动作的请求。

#### AMCL： 定位系统

AMCL(Adaptive Monte Carlo Localization)是移动机器人二维环境下的概率定位系统。它实现了自适应的蒙特卡罗定位方法，其中针对已有的地图使用粒子滤波器跟踪一个机器人的姿态。

#### Planner： 规划器

对于根据传感器信息建立的坐标系，和任务给定的地图与路线，使用GlobalPlanner通过Dijkstra最优路径的算法，对整个过程的路线进行规划，计算costmap上的最小花费路径，作为机器人的全局路线。小车可以在全局规划的过程中自动躲避障碍物，而不影响全局路径。由于传感器可以动态更新costmap，障碍物可以是动态的（比如人从前方走过）。

使用LocalPlanner对未来若干秒内的段距离路线进行规划，其中动态窗口算法（Dynamic Window Approaches）在局部避障中有较好效果，该算法的过程为：

- 采样机器人当前的状态（dx,dy,dtheta）
- 针对每个采样的速度，计算机器人以该速度行驶一段时间后的状态，得出一条行驶的路线。
- 利用一些评价标准为多条路线打分。
- 根据打分，选择最优路径，并重复上面过程。

#### move_base： 导航库

move_base提供了ROS导航的配置、运行、交互接口，经由它来将程序运行的规划最终转换成速度和方向指令。

#### cmd_vel： 运动控制

cmd_vel是一类Twist消息，负责将各个程序对速度的控制指令通过base_controller直接传递给底盘。子消息有linear和angular以反映角速度和线速度，可以给出机器人唯一的运动状态。

#### odom： 里程计

odometry消息，是小车的里程，指通过传感器获得的运动的距离，而实际使用的里程信息是由编码器数据结合惯导经扩展卡尔曼滤波得到。

#### imu：惯性测量单元

是测量物体三轴姿态角以及加速度的装置。来测量物体在三维空间中的角速度和加速度，并以此解算出物体的姿态。作为AHRS(Attitude and Heading Reference System)为搭载ROS的树莓派提供惯性数据，并通过ekf纠正odom的积累误差。

#### ekf：扩展卡尔曼滤波

用于评估小车的位姿，使用了来自不同源的位姿测量信息，它使用带有6自由度模型信息的扩展卡尔曼滤波器来整合来自轮子里程计，IMU传感器和视觉里程计的数据信息。基本思路是用松耦合方式融合不同传感器信息实现位姿估计。

#### base_controller：基本控制节点

是ROS中的基本控制节点，其中包含基本PID算法来控制电机运动。

#### motor：小车底盘

小车底盘，由Arduino驱动，其只是作为一个单纯的硬件控制器来使用，而所有的运算逻辑都放在运行ROS的树莓派上进行，通过串口指令的方式控制Arduino程序的运行。

同时，这个底盘控制驱动也能读取传感器的值以及控制舵机的工作。