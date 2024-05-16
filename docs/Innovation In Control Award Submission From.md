# 控制奖申请表

<p float="left">
<img src="media/team name and num.png" style="width:84%">
<img src="media/队标源文件.png" style="width:15%">
</p>

#### 自动阶段目标 Autonomous Objectives
我们的目标是在自动阶段：
- 首先预装一个紫色像素和一个黄色像素，并通过AprilTag实现场地导航技术将像素放置在背景板上
- 然后回到像素仓库夹取两个像素放置在背景板上
- 随后再次回到像素仓库实行相同操作，将像素放置在背景板上
#### 使用的传感器 Sensors Used
- **自动逼近功能**
    1. 前视距离传感器
    2. 哈士奇视觉传感器
    3. 编码器轮
    4. 惯性导航模块
- **自动阶段导航**
    1. 编码器轮
    2. 惯性导航模块
- **自动阶段定位像素堆**
    1. 巡线传感器
    2. 后视距离传感器
#### 关键算法 Key Algorithms
- **从动轮导航系统**
    1. 竖直方向方向位移计算
    2. 旋转量计算
    3. 速度计算
    4. 平移量计算
- **自动瞄准系统**

#### 操作控制优化 Driver Controlled Enhancements

#### 工程笔记摘要的参考 Engineering Portfolio References

#### 图解自动模式 Autonomous Program Diagrams:
![alt text](<media/auto diagram.png>)