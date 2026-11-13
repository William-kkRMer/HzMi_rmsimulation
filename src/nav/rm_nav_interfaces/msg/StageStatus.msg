# 导航分层状态（L0 感知 / L1 全局 / L2 局部 / L3 控制）
# 每层只报"我这一层好不好"，便于单独定位问题，不必整链推断。

std_msgs/Header header

uint8 stage          # 0=L0 感知, 1=L1 全局, 2=L2 局部, 3=L3 控制
uint8 state          # 0=UNKNOWN 1=OK 2=DEGRADED 3=FAILED
string detail        # 人类可读的一句话（如 "占用率 100% 不合理，本帧丢弃"）

# 该层的关键数值（键值对，长度相同）
string[] keys
float64[] values

uint16 latency_ms    # 本层最近一次处理耗时
