# protocol

这一层处理“字节流和协议帧”，不负责具体控制策略。

当前内容：
- `proto_vofa_firewater.*`：VOFA+ FireWater 文本发包
- `proto_k230.*`：K230 目标偏差轻量帧解析

约束：
- 可以依赖 `common` 和 `bsp_uart`
- 只做封包、解包、状态缓存
- 不把协议解析和云台/底盘控制耦合在一起
