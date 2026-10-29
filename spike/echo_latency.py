#!/usr/bin/env python3
"""Phase 0 spike: echo round-trip latency through the Pico.

Publishes N Int32 values (the value is the sequence number) on /spike/echo_in,
records the send time of each, and matches them against /spike/echo_out.
Prints p50 / p95 / max RTT and the loss count.

usage (inside ros:jazzy-ros-base, --net=host --ipc=host):
    python3 echo_latency.py [count=200] [rate_hz=20]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32


def percentile(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = min(len(sorted_vals) - 1, int(round(p / 100.0 * (len(sorted_vals) - 1))))
    return sorted_vals[k]


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    rclpy.init()
    node = Node("spike_echo_latency")
    sent = {}
    rtts = {}
    base = int(time.time()) % 100000 * 1000  # distinct values per run

    def on_echo(msg):
        t = time.monotonic()
        seq = msg.data - base
        if seq in sent and seq not in rtts:
            rtts[seq] = (t - sent[seq]) * 1000.0

    pub = node.create_publisher(Int32, "/spike/echo_in", qos_profile_sensor_data)
    node.create_subscription(Int32, "/spike/echo_out", on_echo, qos_profile_sensor_data)

    # wait for discovery of the agent-side endpoints
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and (pub.get_subscription_count() == 0
                                           or node.count_publishers("/spike/echo_out") == 0):
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1.0)

    period = 1.0 / rate
    for i in range(count):
        sent[i] = time.monotonic()
        pub.publish(Int32(data=base + i))
        next_t = sent[i] + period
        while time.monotonic() < next_t:
            rclpy.spin_once(node, timeout_sec=max(0.0, next_t - time.monotonic()))
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    vals = sorted(rtts.values())
    print(f"sent={count} received={len(vals)} lost={count - len(vals)}")
    print(f"rtt_ms p50={percentile(vals, 50):.1f} p95={percentile(vals, 95):.1f} "
          f"max={max(vals) if vals else float('nan'):.1f} min={min(vals) if vals else float('nan'):.1f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
