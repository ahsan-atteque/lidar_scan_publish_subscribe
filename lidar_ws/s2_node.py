#!/usr/bin/env python3
"""
ROS 2 node publishing sensor_msgs/LaserScan from an RPLIDAR S2.

DESIGN: two threads.

  Reader thread   Blocks on serial I/O, parses revolutions, hands each
                  completed scan to the publisher via a one-slot queue.
  Executor thread Owns the ROS context: publishes, serves parameters,
                  responds to lifecycle and shutdown.

Serial reads are blocking and a full revolution takes ~100 ms. Doing that
work inside an rclpy timer callback would stall the executor for the whole
duration, delaying parameter services and making Ctrl+C sluggish. Splitting
them keeps the ROS side responsive regardless of what the sensor is doing.

The queue has maxsize=1 and drops the oldest entry when full. For a live
sensor feed a stale scan is worthless -- if the publisher falls behind, the
correct behaviour is to skip ahead, not to build a backlog that grows without
bound and reports increasingly wrong data.
"""

from __future__ import annotations

import math
import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

from rplidar_s2_driver.s2_protocol import RPLidarS2, RPLidarError


class S2Node(Node):

    def __init__(self):
        super().__init__("rplidar_s2")

        # --- Parameters ------------------------------------------------
        self.declare_parameter("serial_port", "/dev/rplidar")
        self.declare_parameter("serial_baudrate", 1000000)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("topic", "scan")
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("inverted", False)
        self.declare_parameter("angle_compensate", True)
        self.declare_parameter("samples_per_scan", 1600)
        self.declare_parameter("min_quality", 0)

        p = self.get_parameter
        self._port = p("serial_port").value
        self._baud = int(p("serial_baudrate").value)
        self._frame_id = p("frame_id").value
        self._topic = p("topic").value
        self._range_min = float(p("range_min").value)
        self._range_max = float(p("range_max").value)
        self._inverted = bool(p("inverted").value)
        self._compensate = bool(p("angle_compensate").value)
        self._bins = int(p("samples_per_scan").value)
        self._min_quality = int(p("min_quality").value)

        # --- Publisher --------------------------------------------------
        # BEST_EFFORT matches how every standard LiDAR driver publishes and
        # how RViz's LaserScan display subscribes by default. RELIABLE here
        # would make RViz silently fail to match until the user changes its
        # QoS by hand -- a confusing failure with no error message.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, self._topic, qos)

        # --- Device -----------------------------------------------------
        self._lidar = RPLidarS2(self._port, self._baud)
        self._queue: "queue.Queue[list]" = queue.Queue(maxsize=1)
        self._running = False
        self._reader: threading.Thread | None = None

        self._connect()

        # Publish from a timer at roughly 2x the sensor rate. Oversampling
        # the queue means we never add latency waiting for the next tick;
        # if no scan is ready the callback returns immediately.
        self._timer = self.create_timer(0.05, self._publish_pending)

    # ------------------------------------------------------------------

    def _connect(self) -> None:
        self._lidar.open()

        info = self._lidar.get_info()
        self.get_logger().info(
            "RPLIDAR model=%#x firmware=%d.%02d hardware=%d"
            % (info["model"], info["firmware_major"],
               info["firmware_minor"], info["hardware"]))
        self.get_logger().info("serial: %s" % info["serial"])

        health = self._lidar.get_health()
        self.get_logger().info(
            "health: %s (code=%d, error=%d)"
            % (health["status"], health["code"], health["error_code"]))
        if health["code"] == 2:
            # ERROR state persists across sessions until explicitly reset.
            self.get_logger().warn("device in ERROR state -- issuing RESET")
            self._lidar.reset()

        # Motor before scan. The rotor needs time to reach stable speed;
        # samples taken during spin-up have wrong angular spacing.
        self._lidar.set_motor(True)
        self.get_logger().info("motor on, waiting for spin-up")
        rclpy.spin_once(self, timeout_sec=1.5)

        self._lidar.start_scan()
        self._running = True
        self._reader = threading.Thread(
            target=self._read_loop, name="s2-reader", daemon=True)
        self._reader.start()
        self.get_logger().info("scanning, publishing on '%s'" % self._topic)

    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """Reader thread: pull revolutions off the wire into the queue."""
        try:
            for scan in self._lidar.iter_scans():
                if not self._running:
                    break
                try:
                    self._queue.put_nowait(scan)
                except queue.Full:
                    # Publisher is behind. Discard the stale scan and keep
                    # the newest -- for live sensor data, freshness beats
                    # completeness.
                    try:
                        self._queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._queue.put_nowait(scan)
                    except queue.Full:
                        pass
        except RPLidarError as e:
            self.get_logger().error("lidar protocol error: %s" % e)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("reader thread died: %s" % e)
        finally:
            self._running = False

    # ------------------------------------------------------------------

    def _publish_pending(self) -> None:
        try:
            scan = self._queue.get_nowait()
        except queue.Empty:
            return
        msg = self._build_msg(scan)
        if msg is not None:
            self._pub.publish(msg)

    def _build_msg(self, points: list):
        """Convert a revolution of ScanPoints into a LaserScan message.

        LaserScan assumes uniform angular spacing: the consumer reconstructs
        each ray's bearing as angle_min + i * angle_increment. The S2 does not
        deliver uniformly spaced samples, so with angle_compensate enabled we
        bin measurements into a fixed-size array indexed by true bearing.
        Without it, we sort by angle and hand over the raw sequence, which is
        faster but slightly distorts geometry -- visible as walls that bow.
        """
        if not points:
            return None

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.range_min = self._range_min
        msg.range_max = self._range_max

        if self._compensate:
            n = self._bins
            ranges = [float("inf")] * n
            intensities = [0.0] * n

            for pt in points:
                if pt.quality < self._min_quality:
                    continue
                d = pt.distance_mm / 1000.0
                if d <= 0.0 or d < self._range_min or d > self._range_max:
                    continue

                a = 360.0 - pt.angle_deg if self._inverted else pt.angle_deg
                idx = int(round(a * n / 360.0)) % n

                # Nearest return wins on collision. Two samples landing in the
                # same bin means one is a grazing/multipath return; the closer
                # one is the real surface, and keeping it fails safe for
                # obstacle avoidance.
                if d < ranges[idx]:
                    ranges[idx] = d
                    intensities[idx] = float(pt.quality)

            msg.angle_min = -math.pi
            msg.angle_max = math.pi
            msg.angle_increment = 2.0 * math.pi / n
            msg.ranges = ranges
            msg.intensities = intensities
        else:
            pts = sorted(points, key=lambda q: q.angle_deg)
            if self._inverted:
                pts = list(reversed(pts))

            ranges = []
            intensities = []
            for pt in pts:
                d = pt.distance_mm / 1000.0
                if (pt.quality < self._min_quality or d <= 0.0
                        or d < self._range_min or d > self._range_max):
                    ranges.append(float("inf"))
                    intensities.append(0.0)
                else:
                    ranges.append(d)
                    intensities.append(float(pt.quality))

            msg.angle_min = math.radians(pts[0].angle_deg) - math.pi
            msg.angle_max = math.radians(pts[-1].angle_deg) - math.pi
            span = msg.angle_max - msg.angle_min
            msg.angle_increment = span / max(1, len(pts) - 1)
            msg.ranges = ranges
            msg.intensities = intensities

        # scan_time / time_increment let downstream consumers (SLAM, Nav2)
        # correct for motion distortion during the sweep. Leaving them zero
        # makes scan matching worse the faster the robot drives.
        msg.scan_time = 0.1  # S2 nominal 10 Hz
        msg.time_increment = msg.scan_time / max(1, len(points))
        return msg

    # ------------------------------------------------------------------

    def destroy_node(self) -> bool:
        """Stop the motor before the process exits.

        Without this the rotor keeps spinning after Ctrl+C -- it draws
        current, and the device is left mid-scan so the NEXT session's
        GET_INFO gets swallowed by the leftover data stream.
        """
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=2.0)
        try:
            self._lidar.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = S2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RPLidarError as e:
        print("failed to start lidar: %s" % e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
