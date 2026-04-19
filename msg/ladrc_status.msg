# LADRC 控制器内部状态调试消息
uint64 timestamp          # 系统启动以来的时间 (微秒)

float32 v                 # 期望输入信号
float32 v1                # TD 跟踪信号
float32 v2                # TD 微分信号 (期望加速度)

float32 e1                # 状态误差 1 (z1 - v1)
float32 e2                # 状态误差 2 (z2 - v2)
float32 u0                # 组合控制律输出
float32 u                 # 最终控制器输出 (包含扰动补偿)

float32 y                 # 观测器参考输入 (实际测量值)
float32 z1                # ESO 状态 1 (估算位置/速度)
float32 z2                # ESO 状态 2 (估算加速度)
float32 z3                # ESO 状态 3 (总扰动估算)
float32 disturb           # 最终转换后的扰动补偿量

# TOPICS ladrc_status_ratex ladrc_status_ratey ladrc_status_ratez
