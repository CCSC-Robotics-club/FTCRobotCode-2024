# 程序部分
### 从动轮导航系统
<p float="left">
  <img src="media/drivenPully1.png" width="42%" />
  <img src="media/drivenPully2.png" width="57%" />
</p>

- 我们使用了从动轮来感知机器的运动，计算机器的位置、速度、加速度等。
每个从动轮上配有编码器，可以读取从动轮转了多少圈，以此推算出从动轮的位移
- 但是，从动轮的位移并不一定就是机器的位移，机器的旋转也会让从动轮转动，这就会造成误差。我们的解决方案是：

1. **使用对称、平行排列的竖直从动轮求平均值消除误差。**
2. **平行从动轮做差得出旋转量，推算出横向从动轮的误差，并从原始数值中减掉**

### 自动阶段路径规划
- 我们将FRC中的PathPlanner下放到了机器当中。PathPlanner是由FRC Team 3015创建的FRC运动轮廓生成器。其优点在于：
1. **我们可以精确地对每条路径进行微调，其每条路径都是用贝塞尔曲线制作的**
<img src="media/spline curve+pathplanner1.png">
<img src="media/pathplanner 2+3.png">
</p>