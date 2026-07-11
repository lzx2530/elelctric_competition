# app

这一层是整车应用层，把驱动、算法和协议按调度节拍串起来。

当前内容：
- `app_control_scheduler.*`：1 kHz 控制节拍和软件分频
- `app_chassis.*`：底盘速度环与巡线逻辑
- `app_imu.*`：IMU 采样与姿态融合输出
- `app_turret.*`：视觉误差到步进频率的闭环
- `app_ui.*`：OLED 页面刷新
- `app_isr.c`：中断分发

约束：
- 允许组合 `drivers / algo / protocol / bsp`
- 不直接散写寄存器访问
- 主循环只做任务编排，不堆积实现细节
