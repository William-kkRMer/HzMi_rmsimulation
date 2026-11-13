"""synth_cloud_publisher.py —— 合成"地平 + 一个 30 cm 高方块"的点云，按 10 Hz 发到指定话题。

用途（**不需要 Gazebo 就能验 native 链路**）：
    gz 原生 gpu_lidar 在无 GL 的环境里起不来（实测 X Error 152 GLX），
    但"gz → 桥 → pc2_to_custom → 感知/LIO"这一段可以完全用合成点云验证。

坐标系 = livox_frame（雷达离地 0.188 m）⇒ 地面 z=-0.188，方块顶面 z=-0.188+0.30。
0.1 m 网格铺 ±6 m（14,400 点）；方块 x∈[1.5,2.5] y∈[-0.5,0.5] ≈ 100 点。

用法：
    python3 tools/scripts/gzsim/synth_cloud_publisher.py --topic /gz/lidar --seconds 15
"""
import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
ap=argparse.ArgumentParser()
ap.add_argument('--topic', default='/gz/lidar')
ap.add_argument('--seconds', type=float, default=15.0)
a=ap.parse_args()
rclpy.init(); n=Node('synth_cloud_publisher')
pub=n.create_publisher(PointCloud2, a.topic, qos_profile_sensor_data)
rng=np.random.default_rng(0)
NX,NY=120,120
xs=np.linspace(-6,6,NX); ys=np.linspace(-6,6,NY)
gx,gy=np.meshgrid(xs,ys)
zs=np.full_like(gx,-0.188)+rng.normal(0,0.003,gx.shape)
# 方块：x∈[1.5,2.5], y∈[-0.5,0.5]，高 0.30 ⇒ 顶面 z=-0.188+0.30
m=(gx>1.5)&(gx<2.5)&(gy>-0.5)&(gy<0.5)
zs[m]=-0.188+0.30
pts=np.stack([gx.ravel(),gy.ravel(),zs.ravel()],1).astype(np.float32)
n_pts=len(pts)
fields=[PointField(name=nm,offset=4*i,datatype=7,count=1) for i,nm in enumerate(['x','y','z','intensity'])]
data=np.hstack([pts, np.full((n_pts,1), 100, np.float32)]).astype(np.float32).tobytes()
msg=PointCloud2(); msg.header.frame_id='livox_frame'
msg.height=1; msg.width=n_pts; msg.fields=fields; msg.point_step=16; msg.row_step=16*n_pts
msg.is_dense=True; msg.data=data
print('合成点云: %d 点（方块 %d 点），z 范围 [%.3f, %.3f]' % (n_pts, int(m.sum()), zs.min(), zs.max()))
t0=time.time()
while time.time()-t0 < a.seconds:
    msg.header.stamp=n.get_clock().now().to_msg()
    pub.publish(msg); time.sleep(0.1)
