"""Stream sensor_msgs/LaserScan data to a live web page served from the robot.

The node subscribes to a LaserScan topic, converts each scan to compact JSON
and pushes it to every connected browser with Server-Sent Events (SSE).
The HTTP server comes from the Python standard library, so nothing beyond
rclpy has to be installed on the Pi.

Endpoints:
    /        the viewer page (web/index.html from the package share directory)
    /stream  SSE stream, one "data: <json>" event per (rate-limited) scan
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Seconds between SSE comments sent while no new scan arrives. They keep
# proxies from closing the connection and let us notice closed browser tabs.
KEEPALIVE_PERIOD_S = 2.0

# Weight of the newest sample in the exponential moving average of scan rate.
SCAN_RATE_SMOOTHING = 0.2


# --------------------------------------------------------------------------
# Scan data: conversion and thread-safe hand-over to the HTTP threads
# --------------------------------------------------------------------------

class ScanHub:
    """Hold the latest encoded scan and wake HTTP threads when it changes.

    The ROS callback (executor thread) publishes into the hub and every
    browser connection (one HTTP thread each) waits on it. A version counter
    lets each connection send every scan at most once.
    """

    def __init__(self):
        """Create an empty hub; no scan is available until publish() is called."""
        self._cond = threading.Condition()
        self._version = 0
        self._payload = None
        self.closed = False

    def publish(self, payload):
        """Store a new encoded scan and wake every waiting connection.

        Args:
            payload (bytes): UTF-8 JSON produced by scan_to_json().
        """
        with self._cond:
            self._payload = payload
            self._version += 1
            self._cond.notify_all()

    def wait_for_newer(self, seen_version, timeout):
        """Block until a scan newer than seen_version exists or timeout passes.

        Args:
            seen_version (int): version the caller already sent (-1 for none).
            timeout (float): maximum wait in seconds.

        Returns:
            tuple[int, bytes | None]: (version, payload). payload is None if
                nothing new arrived before the timeout or the hub was closed;
                version is then unchanged.
        """
        with self._cond:
            self._cond.wait_for(
                lambda: self.closed or (
                    self._payload is not None and self._version != seen_version),
                timeout)
            if self.closed or self._payload is None or self._version == seen_version:
                return seen_version, None
            return self._version, self._payload

    def close(self):
        """Mark the hub closed and release every waiting connection (used at shutdown)."""
        with self._cond:
            self.closed = True
            self._cond.notify_all()


def scan_to_json(msg, scan_rate_hz):
    """Convert a LaserScan into the compact JSON the web page draws.

    Readings that are NaN, infinite or outside [range_min, range_max] are
    sent as null so the page can skip them. Ranges are rounded to
    millimetres to keep each message small (about 3 kB for an LD19 scan).

    Args:
        msg (sensor_msgs.msg.LaserScan): scan to encode. Angles are in
            radians, 0 = robot forward (+x), counter-clockwise positive.
        scan_rate_hz (float): measured rate of incoming scans, in Hz.

    Returns:
        bytes: UTF-8 encoded JSON object with keys frame_id, angle_min,
            angle_increment, range_max, scan_rate_hz and ranges (metres).
    """
    ranges = [
        round(r, 3) if math.isfinite(r) and msg.range_min <= r <= msg.range_max else None
        for r in msg.ranges
    ]
    data = {
        'frame_id': msg.header.frame_id,
        'angle_min': msg.angle_min,
        'angle_increment': msg.angle_increment,
        'range_max': msg.range_max,
        'scan_rate_hz': round(scan_rate_hz, 2),
        'ranges': ranges,
    }
    return json.dumps(data, separators=(',', ':')).encode('utf-8')


# --------------------------------------------------------------------------
# HTTP side: viewer page and SSE stream
# --------------------------------------------------------------------------

class ScanRequestHandler(BaseHTTPRequestHandler):
    """Serve the viewer page and the SSE scan stream.

    Expects self.server to carry two attributes set by ScanWebServer:
    `hub` (ScanHub) and `page` (bytes of index.html).
    """

    def do_GET(self):
        """Route a GET request to the page, the stream, or a 404."""
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            self._send_page()
        elif path == '/stream':
            self._send_stream()
        else:
            self.send_error(404)

    def _send_page(self):
        """Send the cached viewer HTML page."""
        page = self.server.page
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(page)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(page)

    def _send_stream(self):
        """Push scans to this client as SSE events until it disconnects.

        Runs for the lifetime of the browser tab, in its own HTTP thread.
        A closed tab is detected by the next failed write (scan or keep-alive).
        """
        hub = self.server.hub
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        version = -1
        try:
            while not hub.closed:
                version, payload = hub.wait_for_newer(version, KEEPALIVE_PERIOD_S)
                if payload is None:
                    self.wfile.write(b': keep-alive\n\n')
                else:
                    self.wfile.write(b'data: ' + payload + b'\n\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # Browser tab closed; nothing to clean up.

    def log_message(self, format, *args):
        """Silence the default per-request log lines (they flood the console)."""


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------

class ScanWebServer(Node):
    """ROS 2 node that subscribes to LaserScan and serves the live viewer.

    Parameters:
        scan_topic (str): LaserScan topic to show. Default '/scan'.
        host (str): address to bind the web server to. Default '0.0.0.0'
            (all interfaces, so other devices on the network can connect).
        port (int): web server TCP port, 1-65535. Default 8080.
        max_rate_hz (float): maximum scans per second sent to browsers;
            extra scans are dropped to save Pi CPU and Wi-Fi. Default 10.0.
    """

    def __init__(self):
        """Read parameters, load the page, start the HTTP thread and subscribe.

        Raises:
            FileNotFoundError: if web/index.html is not installed.
            OSError: if the port is already in use or not allowed.
        """
        super().__init__('scan_web_server')
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        host = self.declare_parameter('host', '0.0.0.0').value
        port = self.declare_parameter('port', 8080).value
        max_rate_hz = self.declare_parameter('max_rate_hz', 10.0).value

        self._min_send_interval = 1.0 / max_rate_hz if max_rate_hz > 0 else 0.0
        self._last_sent = 0.0
        self._last_received = None
        self._scan_rate_hz = 0.0
        self._hub = ScanHub()

        page_path = os.path.join(
            get_package_share_directory('my_robot_viz'), 'web', 'index.html')
        with open(page_path, 'rb') as f:
            page = f.read()

        try:
            self._httpd = ThreadingHTTPServer((host, port), ScanRequestHandler)
        except OSError as e:
            self.get_logger().error(f'Cannot start web server on {host}:{port}: {e}')
            raise
        self._httpd.daemon_threads = True
        self._httpd.hub = self._hub
        self._httpd.page = page
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'Showing {scan_topic} at http://<this-machine-ip>:{port}/ '
            f'(max {max_rate_hz:g} Hz)')

    def _on_scan(self, msg):
        """Update the measured scan rate and publish the scan if it is due.

        Args:
            msg (sensor_msgs.msg.LaserScan): incoming scan.
        """
        now = time.monotonic()
        if self._last_received is not None:
            dt = now - self._last_received
            if dt > 0:
                rate = 1.0 / dt
                self._scan_rate_hz = (rate if self._scan_rate_hz == 0.0 else
                                      self._scan_rate_hz
                                      + SCAN_RATE_SMOOTHING * (rate - self._scan_rate_hz))
        self._last_received = now

        if now - self._last_sent < self._min_send_interval:
            return
        self._last_sent = now
        self._hub.publish(scan_to_json(msg, self._scan_rate_hz))

    def stop_server(self):
        """Release browser connections and stop the HTTP server."""
        self._hub.close()
        self._httpd.shutdown()
        self._httpd.server_close()


def main(args=None):
    """Run the scan_web_server node until Ctrl+C or ROS shutdown.

    Args:
        args (list[str] | None): command-line arguments for rclpy.init().
    """
    rclpy.init(args=args)
    node = ScanWebServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
