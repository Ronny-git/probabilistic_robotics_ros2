#!/usr/bin/env python3
import math
import signal
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


topics = ['/kf/pose', '/ekf/pose', '/pf/pose']

class PlotFilterResults(Node):
    def __init__(self):
        super().__init__('plot_filter_results')
        self.data = defaultdict(list)
        self.start_time = None

        for topic in topics:
            self.create_subscription(
                PoseStamped,
                topic,
                self.create_callback(topic),
                10)

    def create_callback(self, topic):
        def callback(msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.start_time is None:
                self.start_time = t
            self.data[topic].append((t - self.start_time, msg.pose.position.x, msg.pose.position.y))
        return callback

    def save_plot(self, filename='filter_results.png'):
        if not self.data:
            self.get_logger().warn('No data received from filter topics.')
            return

        fig, ax = plt.subplots(figsize=(8, 6))
        for topic, samples in self.data.items():
            x = [p[1] for p in samples]
            y = [p[2] for p in samples]
            label = topic.replace('/', '')
            ax.plot(x, y, marker='o', linestyle='-', label=label)

        ax.set_title('Filter pose estimates')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(filename)
        self.get_logger().info(f'Saved plot to {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = PlotFilterResults()

    def shutdown(sig, frame):
        node.get_logger().info('Shutting down, saving plot...')
        node.save_plot()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        shutdown(None, None)


if __name__ == '__main__':
    main()
