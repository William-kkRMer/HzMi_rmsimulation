# 规划器的完整输出：时间参数化轨迹 + 安全走廊
# 由 rm_tdt_planner 发布；rm_mpc_controller 消费。
# 约定：所有坐标在 map 系；t 为相对本 plan 起点的秒数（单调递增）。

std_msgs/Header header

# ---- 时间参数化轨迹（并行数组，长度 = N）----
float64[] t
float64[] x
float64[] y
float64[] vx
float64[] vy

# ---- 安全走廊：每 4 个数一个 box（xmin, ymin, xmax, ymax），长度 = 4 * M ----
float32[] corridor

# ---- 元信息 ----
uint16 traj_source        # 0=kinodynamic, 1=minimum_snap, 2=astar+梯形时间, 3=fallback
uint16 corridor_shrink_mm # 走廊收缩量（毫米）
uint16 grid_resolution_mm # 规划栅格分辨率（毫米）
