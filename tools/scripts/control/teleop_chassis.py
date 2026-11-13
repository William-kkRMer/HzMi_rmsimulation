#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""麦轮底盘键盘遥控 —— **打印/键位与旧版 improved_teleop.sh 一致**，内核不再卡

为什么要这个版本：
    旧 shell 版每按一次键都要 `kill` + 重启 `ros2 topic pub -r 10`（ros2 命令行启动 +
    DDS 发现 0.5~1.5 s）⇒ 手感迟钝；但它的**打印**是大家习惯的那一套。
    本文件保留那套打印，内核换成：单 publisher + 50 Hz 定时器 + 非阻塞读键
    ⇒ 按键到动作 < 20 ms。

话题：默认 `/cmd_vel`（上游；由 fake_vel_transform 转发到 /cmd_vel_chassis）。
      fake_vel_transform 只在收到 `/cmd_vel` 时才转发（源码 fake_vel_transform.cpp:82/98），
      空闲不发 ⇒ 走 `/cmd_vel` 不会和它冲突。
      直连底盘：-p topic:=/cmd_vel_chassis
注意：麦轮插件有命令超时（1.2 s）⇒ 本脚本持续发 50 Hz 正好喂饱它；退出时补发零速。

键位（与旧版一致）：
    ↑/↓ 或 i/k   前进 / 后退        ←/→ 或 j/l   左转 / 右转
    空格         停                  >/<          线速度 ±0.1（0.1 ~ 2.0 m/s）
    . / ,        角速度 ±0.1（0.2 ~ 3.0 rad/s）    0   速度复位
    s / h        系统状态 / 帮助      q   停车退出
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BLUE, GREEN, YELLOW, RED, NC = '\033[0;34m', '\033[0;32m', '\033[1;33m', '\033[0;31m', '\033[0m'


class TeleopChassis(Node):
    def __init__(self):
        super().__init__('teleop_chassis')
        # ★ 2026-10-31：**同时**发 /cmd_vel（上游，经 fake_vel_transform）和
        #   /cmd_vel_chassis（直连底盘）。两路值完全相同 ⇒ 不管哪条链活着，底盘都收得到，
        #   不会再出现"指令发了车不动"。也可以 -p topic:=/cmd_vel 只发一路。
        self.topic = self.declare_parameter('topic', '/cmd_vel,/cmd_vel_chassis').value
        self.rate = float(self.declare_parameter('rate', 50.0).value)
        self.lin = float(self.declare_parameter('linear_speed', 0.5).value)
        self.ang = float(self.declare_parameter('angular_speed', 1.0).value)
        self.lin_def, self.ang_def = self.lin, self.ang
        self.vx = 0.0
        self.wz = 0.0
        self.topics = [t.strip() for t in str(self.topic).split(',') if t.strip()]
        self.pubs = [self.create_publisher(Twist, t, 10) for t in self.topics]
        self.create_timer(1.0 / self.rate, self._tick)

    def _tick(self):
        if not rclpy.ok():
            return
        # ★ 2026-10-31 安全保护：**5 s 没按键就自动停车**。
        #   为什么需要：本脚本是 50 Hz **持续发布**，如果用户按了 ← 之后直接关终端/切窗口，
        #   进程还活着就会一直发"左转" ⇒ 下次起仿真时**车一启动就在转**（用户遇到的现象）。
        if (self.vx != 0.0 or self.wz != 0.0) and (time.monotonic() - self.last_key) > 5.0:
            self.vx = self.wz = 0.0
            print(f'{YELLOW}⏱  5 s 无按键，自动停车{NC}')
        m = Twist()
        m.linear.x = self.vx
        m.angular.z = self.wz
        for pub in self.pubs:
            try:
                pub.publish(m)
            except Exception:
                pass

    def stop(self):
        self.vx = self.wz = 0.0
        if not rclpy.ok():
            return
        for _ in range(12):
            try:
                self._tick()
            except Exception:      # 上下文已失效 / publisher 已销毁 ⇒ 别再抛
                break

    # ---- 按键：打印风格与旧版一致 ----
    def on_key(self, key: str) -> bool:
        self.last_key = time.monotonic()
        # ★ 方向键规范化（三种终端写法都认）：
        #   普通模式 ESC[A/B/C/D · 应用模式 ESCOA/B/C/D · 带修饰键 ESC[1;2A 之类
        if key.startswith('\x1b'):
            seq = key.replace('\x1bO', '\x1b[')
            last = seq[-1:]
            key = {'A': '\x1b[A', 'B': '\x1b[B', 'C': '\x1b[C', 'D': '\x1b[D'}.get(last, key)
        if key in ('\x1b[A', 'i', 'I'):
            self.vx, self.wz = self.lin, 0.0
            print(f'{BLUE}→ 前进 (线速度: {self.lin}){NC}')
        elif key in ('\x1b[B', 'k', 'K'):
            self.vx, self.wz = -self.lin, 0.0
            print(f'{BLUE}← 后退 (线速度: {self.lin}){NC}')
        elif key in ('\x1b[D', 'j', 'J'):
            self.vx, self.wz = 0.0, self.ang
            print(f'{BLUE}↺ 左转 (角速度: {self.ang}){NC}')
        elif key in ('\x1b[C', 'l', 'L'):
            self.vx, self.wz = 0.0, -self.ang
            print(f'{BLUE}↻ 右转 (角速度: {self.ang}){NC}')
        elif key == ' ':
            self.vx = self.wz = 0.0
            print(f'{YELLOW}⏹ 紧急停止{NC}')
        elif key == '>':
            self.lin = min(2.0, round(self.lin + 0.1, 1))
            print(f'{GREEN}⚡ 线速度 +0.1 → {self.lin} m/s{NC}')
        elif key == '<':
            self.lin = max(0.1, round(self.lin - 0.1, 1))
            print(f'{GREEN}⚡ 线速度 -0.1 → {self.lin} m/s{NC}')
        elif key == '.':
            self.ang = min(3.0, round(self.ang + 0.1, 1))
            print(f'{GREEN}⚡ 角速度 +0.1 → {self.ang} rad/s{NC}')
        elif key == ',':
            self.ang = max(0.2, round(self.ang - 0.1, 1))
            print(f'{GREEN}⚡ 角速度 -0.1 → {self.ang} rad/s{NC}')
        elif key == '0':
            self.lin, self.ang = self.lin_def, self.ang_def
            print(f'{GREEN}⚡ 速度已重置为默认值{NC}')
        elif key in ('s', 'S'):
            self.show_status()
        elif key in ('h', 'H', '?'):
            self.show_help()
        elif key in ('q', 'Q'):
            print(f'{YELLOW}👋 退出（先发零速停车）{NC}')
            return False
        self._tick()          # ★ 立刻发一帧：按键到动作不等定时器
        return True

    def show_status(self):
        n = self.count_subscribers(self.topic)
        print(f'\n{BLUE}📊 系统状态:{NC}')
        ts = ', '.join(self.topics)
        print(f'  {BLUE}控制话题: {ts}{NC}')
        print(f'  {BLUE}当前速度设置:{NC} 线速度 {self.lin} m/s，角速度 {self.ang} rad/s')
        print(f'  当前输出: vx={self.vx:.2f} wz={self.wz:.2f}\n')

    def show_help(self):
        ts_help = ', '.join(self.topics)
        print(f"""
{BLUE}📖 帮助信息:{NC}
  ↑ / ↓       : 前进 / 后退          (兼容 i / k)
  ← / →       : 左转 / 右转          (兼容 j / l)
  空格        : 紧急停止
  > / <       : 线速度 +0.1 / -0.1  (0.1 ~ 2.0 m/s)
  . / ,       : 角速度 +0.1 / -0.1  (0.2 ~ 3.0 rad/s)
  0           : 重置速度
  s           : 系统状态
  h           : 显示帮助
  q           : 退出（自动停）
  当前设置: 线速度 {self.lin} m/s，角速度 {self.ang} rad/s，话题 {ts_help}
""")


def read_key(fd, timeout=0.05):
    """读一个键（或一整条 ESC 序列）。返回**字符串**。

    ★ 2026-10-31 关键修正：必须用 `os.read`，**不能**用 `sys.stdin.read`。
      原因：`sys.stdin` 是**带缓冲**的 Python 流。第一次 `read(1)` 拿到 ESC 时，
      Python 已经把后面的 `[`、`A` 一起吞进**用户态缓冲**了；再去 `select()` 看 fd
      当然"没有数据" ⇒ 收不齐 ⇒ **方向键完全没反应**（用户反馈的真因）。
      改用 `os.read(fd, 8)` 一次把这一波字节全取出来（方向键 3 字节就一起来了）。
    """
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return ''
    try:
        return os.read(fd, 8).decode(errors='ignore')
    except OSError:
        return ''


def main():
    print(f'{BLUE}==============================================={NC}')
    print(f'{BLUE}  🤖 改进版机器人键盘控制{NC}')
    print(f'{BLUE}==============================================={NC}\n')

    rclpy.init()
    node = TeleopChassis()
    print(f'{YELLOW}🔍 检查 ROS2 环境（domain={os.environ.get("ROS_DOMAIN_ID", "0")}，topic={node.topic}）...{NC}')
    time.sleep(0.6)                      # 给 DDS 一点时间做发现
    for t in node.topics:
        n = node.count_subscribers(t)
        if n > 0:
            print(f'{GREEN}✅ {t}：订阅者 {n}{NC}')
        else:
            print(f'{YELLOW}⚠️  {t}：**没有订阅者**（栈没起？或 domain 不一致）{NC}')
    print(f'{GREEN}✅ 系统检查完成{NC}\n')

    print(f'{BLUE}控制说明:{NC}')
    print('  i - 前进    k - 后退')
    print('  j - 左转    l - 右转')
    print('  空格 - 停止  q - 退出')
    print('  s - 系统状态  h - 帮助')
    print('  Ctrl+C - 退出\n')

    print(f'{GREEN}🎮 键盘控制已启动（方向键）{NC}')
    print('  ↑/↓ 前进/后退    ←/→ 左转/右转    空格 停车')
    print('  >/< 线速度±0.1   ./, 角速度±0.1    0 重置   s 状态   h 帮助   q 退出')
    print('  ★ 按一下 = 持续走，空格才停\n')

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        running = True
        while running and rclpy.ok():
            k = read_key(fd)
            if k:
                running = node.on_key(k)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # ★ 先还原终端！原来 stop() 抛异常时 tcsetattr 永远执行不到
        #   ⇒ 终端卡在 cbreak（不回显、方向键/退格失灵）—— 用户看到的就是
        #   "一运行，那个终端前后左右就用不了了"。
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass
        try:
            node.stop()
        except Exception:
            pass
        print(f'{GREEN}✅ 程序退出{NC}')
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
