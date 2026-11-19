#!/usr/bin/env python3
"""강의 26: "로봇 컴퓨터" 의 자원 사용량을 1초마다 알려 주는 ROS 2 노드.

Isaac ROS 컨테이너 안에서 실행한다 (컨테이너가 로봇 컴퓨터 역할).
    python3 computer_monitor.py

- GPU: 사용률, 메모리, 온도 (nvidia-smi)
- CPU: 이 컨테이너가 쓴 양을 코어 수로 나타낸 값, 허용된 코어 수 (docker update --cpus 로 준 제한)
같은 값을 표준 진단 토픽 /diagnostics (diagnostic_msgs/DiagnosticArray) 로도 보낸다 → rqt_runtime_monitor 로 볼 수 있다.
"""
import os
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

CGROUP = "/sys/fs/cgroup"
GPU_QUERY = "name,utilization.gpu,memory.used,memory.total,temperature.gpu"


def read_gpu():
    out = subprocess.run(["nvidia-smi", f"--query-gpu={GPU_QUERY}", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True).stdout.strip().splitlines()[0]
    name, util, used, total, temp = [v.strip() for v in out.split(",")]
    return name, float(util), float(used), float(total), float(temp)


def cpu_limit():
    """허용된 코어 수. 제한이 없으면 PC 의 코어 수."""
    quota, period = open(f"{CGROUP}/cpu.max").read().split()
    return os.cpu_count() if quota == "max" else float(quota) / float(period)


def cpu_used_seconds():
    """이 컨테이너가 지금까지 쓴 CPU 시간 [s]."""
    for line in open(f"{CGROUP}/cpu.stat"):
        if line.startswith("usage_usec"):
            return float(line.split()[1]) / 1e6
    return 0.0


class ComputerMonitor(Node):
    def __init__(self):
        super().__init__("computer_monitor")
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.before = (time.monotonic(), cpu_used_seconds())
        self.create_timer(1.0, self.report)

    def report(self):
        now = (time.monotonic(), cpu_used_seconds())
        cores = (now[1] - self.before[1]) / (now[0] - self.before[0])     # 1초 동안 쓴 코어 수
        self.before = now
        limit = cpu_limit()
        name, util, used, total, temp = read_gpu()

        status = DiagnosticStatus(name="computer_monitor", hardware_id=name)
        busy = cores > 0.9 * limit or util > 90.0
        status.level = DiagnosticStatus.WARN if busy else DiagnosticStatus.OK
        status.message = "busy" if busy else "ok"
        values = {"gpu_util_percent": util, "gpu_memory_used_mb": used, "gpu_memory_total_mb": total,
                  "gpu_temperature_c": temp, "cpu_cores_used": round(cores, 2), "cpu_cores_allowed": limit}
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.pub.publish(msg)
        self.get_logger().info(f"GPU {util:3.0f} %  {used:5.0f} / {total:.0f} MB  {temp:.0f} C  |  "
                               f"CPU {cores:5.2f} / {limit:g} 코어  [{status.message}]")


def main():
    rclpy.init()
    node = ComputerMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
