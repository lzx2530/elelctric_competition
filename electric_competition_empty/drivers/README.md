# drivers

这一层描述“设备”或“功能外设”的行为，基于 `bsp` 组合出更接近业务的控制接口。

当前内容：
- `drv_motor_dc.*`：单路直流电机 `PWM + DIR`
- `drv_stepper.*`：单轴步进 `STEP + DIR`
- `drv_encoder_ab.*`：双路软件 AB 解码
- `drv_line_sensor.*`：8 路数字灰度读取和偏差计算
- `drv_oled_ssd1306.*`：I2C OLED 显示
- `drv_mpu9250.*`：六轴 IMU 初始化、采样和标定

约束：
- 允许依赖 `bsp` 和 `common`
- 不直接写主循环调度和整车策略
- 一个驱动对应一个清晰句柄和一组明确接口
