import getpass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from process_monitor_msgs.msg import ProcessStatArray, ProcessStat

try:
    import psutil
except ImportError:  # pragma: no cover - runtime guard
    psutil = None


class ProcessMonitorNode(Node):
    """Publish per-process CPU and memory statistics for Foxglove's table view."""

    def __init__(self) -> None:
        super().__init__('process_monitor')

        self.declare_parameter('publish_topic', 'system/processes')
        self.declare_parameter('update_period', 0.5)
        self.declare_parameter('max_processes', 50)
        self.declare_parameter('include_all_users', True)

        topic = self.get_parameter('publish_topic').value
        update_period = float(self.get_parameter('update_period').value)
        self._max_processes = int(self.get_parameter('max_processes').value)
        self._include_all_users = bool(self.get_parameter('include_all_users').value)
        self._target_username: Optional[str] = None if self._include_all_users else getpass.getuser()

        if update_period <= 0.0:
            self.get_logger().warning(
                'update_period parameter must be > 0. Using 0.5 s default instead.',
            )
            update_period = 0.5

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._publisher = self.create_publisher(ProcessStatArray, topic, qos)

        self._psutil_available_logged = False
        self._prime_cpu_stats()

        self._timer = self.create_timer(update_period, self._publish_process_stats)
        self.get_logger().info(
            f'Publishing process statistics on {topic} every {update_period:.2f} s '
            f'(max {self._max_processes} processes)',
        )

    def _prime_cpu_stats(self) -> None:
        if psutil is None:
            return

        for proc in psutil.process_iter(['pid']):
            try:
                proc.cpu_percent(None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

    def _publish_process_stats(self) -> None:
        if psutil is None:
            if not self._psutil_available_logged:
                self.get_logger().error(
                    "python3-psutil is not available. Install it to enable process monitoring.",
                )
                self._psutil_available_logged = True
            return

        msg = ProcessStatArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'process_monitor'

        stats: List[ProcessStat] = []
        for proc in psutil.process_iter(['pid', 'name', 'username', 'memory_percent', 'memory_info']):
            if self._target_username is not None:
                username = proc.info.get('username')
                if username != self._target_username:
                    continue

            try:
                cpu_percent = proc.cpu_percent(None)
                mem_percent = float(proc.info.get('memory_percent') or 0.0)
                mem_info = proc.info.get('memory_info')
                rss_bytes = getattr(mem_info, 'rss', 0)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

            memory_mb = rss_bytes / (1024 ** 2)

            stat = ProcessStat()
            stat.name = proc.info.get('name') or 'unknown'
            stat.pid = int(proc.pid)
            stat.cpu_percent = float(cpu_percent)
            stat.memory_percent = float(mem_percent)
            stat.memory_mb = float(memory_mb)
            stat.username = proc.info.get('username') or ''

            stats.append(stat)

        stats.sort(key=lambda item: item.cpu_percent, reverse=True)
        if self._max_processes > 0:
            stats = stats[: self._max_processes]

        msg.processes = stats
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = ProcessMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
