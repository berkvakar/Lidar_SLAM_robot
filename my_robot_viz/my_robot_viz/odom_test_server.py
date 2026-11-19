"""Odometry test bench: run test moves and watch nav_msgs/Odometry in a web page.

The node subscribes to an Odometry topic and streams the robot pose and
velocity to every connected browser with Server-Sent Events (SSE), like
scan_web_server does for LiDAR scans. From the page you can start test moves
(drive a distance, rotate an angle, spin the wheels N revolutions). The node
then publishes geometry_msgs/Twist on cmd_vel and reports how far odometry
says the robot went, so the result can be compared with a tape measure.

Safety:
    * Nothing is published on cmd_vel unless the node runs with
      allow_motion:=true.
    * Every move is timed on the robot, not in the browser, and ends with
      zero-velocity commands. Speeds are capped by max_linear / max_angular.
    * A move is aborted if odometry stops arriving, if the last browser tab
      disconnects, or if the STOP button (or Esc key) is pressed.

fake_odom:=true makes the node publish its own Odometry by integrating the
commanded velocity, so the page and the test moves can be tried before a
real motor/encoder driver exists. Do not use it while a real driver
publishes on the same topic.

Endpoints:
    GET  /        the test page (web/odom.html from the package share directory)
    GET  /stream  SSE stream, one JSON state snapshot per control tick
    POST /move    JSON {kind, amount, speed, pattern, stop_on}: start a test move
    POST /stop    stop the robot now
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Control loop period in seconds (20 Hz). Each tick publishes cmd_vel during
# a move, steps the fake odometry and sends one snapshot to the browsers.
CONTROL_PERIOD_S = 0.05

# Seconds between SSE comments sent while nothing new is published.
KEEPALIVE_PERIOD_S = 2.0

# After a move stops, wait this long (s) before measuring, so the distance
# the robot coasts after the stop command is included in the result.
SETTLE_S = 0.7

# Abort a running move if no odometry arrived for this long (s).
ODOM_TIMEOUT_S = 1.0

# Number of zero-velocity messages sent after a move ends, in case one is lost.
ZERO_REPEAT = 5

# Limits for one test move.
MAX_MOVE_S = 30.0
MAX_DISTANCE_M = 5.0
MAX_ANGLE_DEG = 3600.0
MAX_WHEEL_REVS = 50.0

# A closed-loop move (stop_on='odom') gives up after
# expected_duration * factor + extra seconds.
CLOSED_LOOP_TIMEOUT_FACTOR = 2.0
CLOSED_LOOP_TIMEOUT_EXTRA_S = 2.0

# Largest accepted POST body, in bytes.
MAX_BODY_BYTES = 4096

# Weight of the newest sample in the moving average of the odometry rate.
ODOM_RATE_SMOOTHING = 0.2


# --------------------------------------------------------------------------
# State hand-over between the ROS thread and the HTTP threads
# --------------------------------------------------------------------------

class StateHub:
    """Hold the latest encoded state snapshot and wake HTTP threads when it changes.

    Same pattern as ScanHub in scan_web_server.py, plus a count of connected
    browsers so a move can be aborted when the last tab closes.
    """

    def __init__(self):
        """Create an empty hub with no snapshot and no connected clients."""
        self._cond = threading.Condition()
        self._version = 0
        self._payload = None
        self.closed = False
        self.clients = 0

    def publish(self, payload):
        """Store a new snapshot and wake every waiting connection.

        Args:
            payload (bytes): UTF-8 JSON snapshot.
        """
        with self._cond:
            self._payload = payload
            self._version += 1
            self._cond.notify_all()

    def wait_for_newer(self, seen_version, timeout):
        """Block until a snapshot newer than seen_version exists or timeout passes.

        Args:
            seen_version (int): version the caller already sent (-1 for none).
            timeout (float): maximum wait in seconds.

        Returns:
            tuple[int, bytes | None]: (version, payload). payload is None if
                nothing new arrived before the timeout or the hub was closed.
        """
        with self._cond:
            self._cond.wait_for(
                lambda: self.closed or (
                    self._payload is not None and self._version != seen_version),
                timeout)
            if self.closed or self._payload is None or self._version == seen_version:
                return seen_version, None
            return self._version, self._payload

    def change_clients(self, delta):
        """Add delta (+1 on connect, -1 on disconnect) to the browser count.

        Args:
            delta (int): change in the number of connected browsers.
        """
        with self._cond:
            self.clients += delta

    def close(self):
        """Mark the hub closed and release every waiting connection (used at shutdown)."""
        with self._cond:
            self.closed = True
            self._cond.notify_all()


# --------------------------------------------------------------------------
# Geometry and test-move maths (pure functions, no ROS state)
# --------------------------------------------------------------------------

def yaw_from_quaternion(q):
    """Extract the heading (rotation about z) from a quaternion.

    Args:
        q (geometry_msgs.msg.Quaternion): orientation with fields x, y, z, w.

    Returns:
        float: yaw in radians, in [-pi, pi], counter-clockwise positive.
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi].

    Args:
        angle (float): angle in radians.

    Returns:
        float: the same direction expressed in [-pi, pi] radians.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def displacement(start, end):
    """Compute how far the robot moved between two odometry poses.

    Distances are measured in the robot frame at the start pose, so
    'forward' is progress along the starting heading and 'lateral' is
    sideways drift (positive = to the left).

    Args:
        start (dict): pose with keys x, y (metres), yaw and yaw_total
            (radians; yaw_total is unwrapped, so it keeps counting past 180 deg).
        end (dict): later pose with the same keys.

    Returns:
        tuple[float, float, float]: (forward m, lateral m, turned rad), with
            turned counter-clockwise positive and not wrapped.
    """
    dx = end['x'] - start['x']
    dy = end['y'] - start['y']
    c, s = math.cos(start['yaw']), math.sin(start['yaw'])
    return dx * c + dy * s, -dx * s + dy * c, end['yaw_total'] - start['yaw_total']


def plan_move(request, wheel_radius, wheel_separation, max_linear, max_angular):
    """Validate a test-move request and turn it into a velocity command plan.

    Test kinds (a negative amount reverses the direction):
        drive:  drive straight `amount` metres at `speed` m/s.
        rotate: turn in place `amount` degrees at `speed` deg/s
                (positive = counter-clockwise / left).
        wheels: turn each wheel `amount` revolutions at `speed` rev/s.
                pattern 'straight' turns both wheels forward (robot drives
                straight); 'spin' turns them in opposite directions (robot
                rotates in place, positive = counter-clockwise).

    Args:
        request (dict): JSON from the page with keys kind, amount, speed,
            and optionally pattern ('straight' | 'spin', default 'straight')
            and stop_on ('time' = open loop, stop after the computed
            duration; 'odom' = closed loop, stop when odometry reaches the
            target; default 'time').
        wheel_radius (float): wheel radius in metres, > 0.
        wheel_separation (float): distance between left and right wheels in
            metres, > 0.
        max_linear (float): linear speed cap in m/s, > 0.
        max_angular (float): angular speed cap in rad/s, > 0.

    Returns:
        dict: plan with keys kind, pattern, stop_on, amount, linear (m/s),
            angular (rad/s), duration (s, expected), target_distance (m,
            signed, 0 for turns) and target_angle (rad, signed, 0 for drives).

    Raises:
        ValueError: with a user-readable message if the request is invalid,
            outside the limits, or would take longer than MAX_MOVE_S.
    """
    kind = request.get('kind')
    stop_on = request.get('stop_on', 'time')
    pattern = request.get('pattern', 'straight') if kind == 'wheels' else None
    if stop_on not in ('time', 'odom'):
        raise ValueError("stop_on must be 'time' or 'odom'")
    if pattern not in (None, 'straight', 'spin'):
        raise ValueError("pattern must be 'straight' or 'spin'")
    try:
        amount = float(request.get('amount'))
        speed = abs(float(request.get('speed')))
    except (TypeError, ValueError):
        raise ValueError('amount and speed must be numbers') from None
    if not (math.isfinite(amount) and math.isfinite(speed)) or amount == 0 or speed == 0:
        raise ValueError('amount and speed must be non-zero numbers')

    sign = math.copysign(1.0, amount)
    circumference = 2.0 * math.pi * wheel_radius
    half_track = wheel_separation / 2.0
    linear = angular = target_distance = target_angle = 0.0

    if kind == 'drive':
        if abs(amount) > MAX_DISTANCE_M:
            raise ValueError(f'distance is limited to {MAX_DISTANCE_M:g} m')
        linear = sign * min(speed, max_linear)
        target_distance = amount
    elif kind == 'rotate':
        if abs(amount) > MAX_ANGLE_DEG:
            raise ValueError(f'angle is limited to {MAX_ANGLE_DEG:g} deg')
        angular = sign * min(math.radians(speed), max_angular)
        target_angle = math.radians(amount)
    elif kind == 'wheels':
        if abs(amount) > MAX_WHEEL_REVS:
            raise ValueError(f'wheel turns are limited to {MAX_WHEEL_REVS:g} revolutions')
        wheel_arc = amount * circumference          # metres each wheel rolls
        wheel_speed = speed * circumference         # m/s at the wheel rim
        if pattern == 'straight':
            linear = sign * min(wheel_speed, max_linear)
            target_distance = wheel_arc
        else:
            # Opposite wheels at rim speed u rotate the robot at u / half_track.
            angular = sign * min(wheel_speed / half_track, max_angular)
            target_angle = wheel_arc / half_track
    else:
        raise ValueError(f'unknown test kind {kind!r}')

    if target_distance:
        duration = abs(target_distance / linear)
    else:
        duration = abs(target_angle / angular)
    if duration > MAX_MOVE_S:
        raise ValueError(
            f'this test would take {duration:.1f} s (limit {MAX_MOVE_S:g} s); '
            'raise the speed or shorten it')

    return {
        'kind': kind, 'pattern': pattern, 'stop_on': stop_on, 'amount': amount,
        'linear': linear, 'angular': angular, 'duration': duration,
        'target_distance': target_distance, 'target_angle': target_angle,
    }


def summarize_move(plan, start, end, elapsed, reason, wheel_radius, wheel_separation):
    """Compare what a test move commanded with what odometry measured.

    Args:
        plan (dict): plan from plan_move().
        start (dict): odometry pose when the move started (see displacement()).
        end (dict): odometry pose after the settle time.
        elapsed (float): seconds from start until the stop command.
        reason (str | None): why the move was aborted, or None if it
            finished normally.
        wheel_radius (float): wheel radius in metres, > 0.
        wheel_separation (float): left-right wheel distance in metres, > 0.

    Returns:
        dict: result with the plan's kind, pattern, stop_on and amount, plus
            unit ('m' or 'rad'), commanded and measured (in that unit),
            error_pct (measured vs commanded, %), forward and lateral (m),
            turned (rad), elapsed (s), reason, and for 'wheels' tests
            measured_revs (wheel revolutions implied by odometry).
    """
    forward, lateral, turned = displacement(start, end)
    if plan['target_distance']:
        unit, commanded, measured = 'm', plan['target_distance'], forward
    else:
        unit, commanded, measured = 'rad', plan['target_angle'], turned

    result = {
        'kind': plan['kind'], 'pattern': plan['pattern'], 'stop_on': plan['stop_on'],
        'amount': plan['amount'], 'unit': unit,
        'commanded': round(commanded, 4), 'measured': round(measured, 4),
        'error_pct': round((measured - commanded) / commanded * 100.0, 2),
        'forward': round(forward, 4), 'lateral': round(lateral, 4),
        'turned': round(turned, 4), 'elapsed': round(elapsed, 2), 'reason': reason,
    }
    if plan['kind'] == 'wheels':
        # Arc each wheel rolled according to odometry, divided by circumference.
        wheel_arc = forward if plan['pattern'] == 'straight' else turned * wheel_separation / 2.0
        result['measured_revs'] = round(wheel_arc / (2.0 * math.pi * wheel_radius), 3)
    return result


# --------------------------------------------------------------------------
# HTTP side: test page, SSE stream and move/stop commands
# --------------------------------------------------------------------------

class OdomRequestHandler(BaseHTTPRequestHandler):
    """Serve the test page, the SSE state stream and the move/stop commands.

    Expects self.server to carry three attributes set by OdomTestServer:
    `hub` (StateHub), `page` (bytes of odom.html) and `node` (OdomTestServer).
    """

    def do_GET(self):
        """Route a GET request to the page, the stream, or a 404."""
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html', '/odom.html'):
            page = self.server.page
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(page)
        elif path == '/stream':
            self._send_stream()
        else:
            self.send_error(404)

    def do_POST(self):
        """Handle /move and /stop.

        Only JSON bodies are accepted. A browser can only send that
        content type cross-origin after a CORS preflight, which this server
        never approves, so other websites cannot start moves.
        """
        path = self.path.split('?', 1)[0]
        if path not in ('/move', '/stop'):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            length = -1
        content_type = self.headers.get('Content-Type', '')
        if not 0 <= length <= MAX_BODY_BYTES or not content_type.startswith('application/json'):
            self._send_json(400, {'ok': False, 'error': 'expected a small JSON body'})
            return
        body = self.rfile.read(length)

        node = self.server.node
        if path == '/stop':
            node.request_stop()
            self._send_json(200, {'ok': True})
            return
        try:
            request = json.loads(body or b'{}')
            if not isinstance(request, dict):
                raise ValueError
        except ValueError:
            self._send_json(400, {'ok': False, 'error': 'body must be a JSON object'})
            return
        error = node.request_move(request)
        self._send_json(200 if error is None else 400, {'ok': error is None, 'error': error})

    def _send_json(self, status, data):
        """Send a small JSON response.

        Args:
            status (int): HTTP status code.
            data (dict): object to encode.
        """
        body = json.dumps(data).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(body)

    def _send_stream(self):
        """Push state snapshots to this client as SSE events until it disconnects.

        Runs for the lifetime of the browser tab, in its own HTTP thread, and
        counts as a connected client while it runs.
        """
        hub = self.server.hub
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        hub.change_clients(+1)
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
            pass  # Browser tab closed.
        finally:
            hub.change_clients(-1)

    def log_message(self, format, *args):
        """Silence the default per-request log lines (they flood the console)."""


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------

class OdomTestServer(Node):
    """ROS 2 node that runs odometry test moves and serves the test page.

    Parameters:
        odom_topic (str): nav_msgs/Odometry topic to watch. Default '/odom'.
        cmd_vel_topic (str): geometry_msgs/Twist topic for test moves.
            Default '/cmd_vel'.
        host (str): web server bind address. Default '0.0.0.0'.
        port (int): web server TCP port, 1-65535. Default 8081.
        allow_motion (bool): publish on cmd_vel during tests. Default False
            (view only; tests still run against fake odometry).
        fake_odom (bool): publish simulated Odometry on odom_topic by
            integrating the commanded velocity. Default False.
        wheel_radius (float): wheel radius in metres, used to convert wheel
            revolutions to distance. Default 0.0325 (65 mm TT wheel).
        wheel_separation (float): left-right wheel distance in metres.
            Default 0.14.
        max_linear (float): linear speed cap in m/s. Default 0.3.
        max_angular (float): angular speed cap in rad/s. Default 1.5.
        odom_frame (str), base_frame (str): frame ids written into fake
            odometry. Defaults 'odom' and 'base_footprint'.
    """

    def __init__(self):
        """Read parameters, load the page, start the HTTP thread, publishers and timer.

        Raises:
            ValueError: if wheel_radius, wheel_separation or a speed cap is not > 0.
            FileNotFoundError: if web/odom.html is not installed.
            OSError: if the web server port is in use or not allowed.
        """
        super().__init__('odom_test_server')
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        host = self.declare_parameter('host', '0.0.0.0').value
        port = self.declare_parameter('port', 8081).value
        self._allow_motion = self.declare_parameter('allow_motion', False).value
        self._fake_odom = self.declare_parameter('fake_odom', False).value
        self._wheel_radius = self.declare_parameter('wheel_radius', 0.0325).value
        self._wheel_separation = self.declare_parameter('wheel_separation', 0.14).value
        self._max_linear = self.declare_parameter('max_linear', 0.3).value
        self._max_angular = self.declare_parameter('max_angular', 1.5).value
        self._odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self._base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        if min(self._wheel_radius, self._wheel_separation,
               self._max_linear, self._max_angular) <= 0:
            raise ValueError('wheel_radius, wheel_separation, max_linear and '
                             'max_angular must all be > 0')

        # Everything below the lock is shared with the HTTP threads.
        self._lock = threading.Lock()
        self._pending = None          # plan requested by the page, started next tick
        self._stop_requested = False
        self._move = None             # running/settling move, see _update_move()
        self._result = None           # last finished move, sent with every snapshot
        self._result_id = 0
        self._odom = None             # latest pose: x, y, yaw, yaw_total, v, w
        self._odom_received_at = None
        self._odom_rate_hz = 0.0
        self._cmd = (0.0, 0.0)        # commanded (linear m/s, angular rad/s)
        self._zero_ticks = 0          # zero-velocity messages still to send
        self._fake_pose = [0.0, 0.0, 0.0]  # x m, y m, yaw rad; timer thread only

        self._config = {
            'odom_topic': odom_topic, 'cmd_vel_topic': cmd_vel_topic,
            'allow_motion': self._allow_motion, 'fake_odom': self._fake_odom,
            'wheel_radius': self._wheel_radius, 'wheel_separation': self._wheel_separation,
            'max_linear': self._max_linear, 'max_angular': self._max_angular,
        }

        page_path = os.path.join(
            get_package_share_directory('my_robot_viz'), 'web', 'odom.html')
        with open(page_path, 'rb') as f:
            page = f.read()

        self._cmd_pub = (self.create_publisher(Twist, cmd_vel_topic, 10)
                         if self._allow_motion else None)
        self._fake_pub = (self.create_publisher(Odometry, odom_topic, 10)
                          if self._fake_odom else None)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        self._hub = StateHub()
        try:
            self._httpd = ThreadingHTTPServer((host, port), OdomRequestHandler)
        except OSError as e:
            self.get_logger().error(f'Cannot start web server on {host}:{port}: {e}')
            raise
        self._httpd.daemon_threads = True
        self._httpd.hub = self._hub
        self._httpd.page = page
        self._httpd.node = self
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

        self.create_timer(CONTROL_PERIOD_S, self._on_tick)

        motion = (f'motion ENABLED on {cmd_vel_topic}' if self._allow_motion
                  else 'motion disabled (allow_motion:=false)')
        fake = ', FAKE odometry' if self._fake_odom else ''
        self.get_logger().info(
            f'Odometry test page for {odom_topic} at http://<this-machine-ip>:{port}/ '
            f'({motion}{fake})')

    # -- called from HTTP threads -------------------------------------------

    def request_move(self, request):
        """Queue a test move requested by the page (HTTP thread).

        Args:
            request (dict): move request, see plan_move().

        Returns:
            str | None: user-readable error message, or None if the move was
                queued and will start on the next control tick.
        """
        if not (self._allow_motion or self._fake_odom):
            return 'Motion is disabled. Relaunch with allow_motion:=true (or fake_odom:=true).'
        try:
            plan = plan_move(request, self._wheel_radius, self._wheel_separation,
                             self._max_linear, self._max_angular)
        except ValueError as e:
            return str(e)
        with self._lock:
            if self._move is not None or self._pending is not None:
                return 'A test is already running. Wait for it or press STOP.'
            if (self._odom_received_at is None
                    or time.monotonic() - self._odom_received_at > ODOM_TIMEOUT_S):
                return f'No odometry on {self._config["odom_topic"]}, so the test cannot be measured.'
            self._pending = plan
        return None

    def request_stop(self):
        """Ask the control loop to stop the robot on its next tick (HTTP thread)."""
        with self._lock:
            self._stop_requested = True
            self._pending = None

    # -- ROS callbacks (executor thread) ------------------------------------

    def _on_odom(self, msg):
        """Store the latest odometry pose and update the measured message rate.

        Yaw is also accumulated without wrapping (yaw_total) so turns of
        more than 180 degrees can be measured.

        Args:
            msg (nav_msgs.msg.Odometry): incoming odometry.
        """
        now = time.monotonic()
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self._lock:
            if self._odom is None:
                yaw_total = yaw
            else:
                yaw_total = self._odom['yaw_total'] + wrap_angle(yaw - self._odom['yaw'])
            if self._odom_received_at is not None and now > self._odom_received_at:
                rate = 1.0 / (now - self._odom_received_at)
                self._odom_rate_hz = (rate if self._odom_rate_hz == 0.0 else
                                      self._odom_rate_hz
                                      + ODOM_RATE_SMOOTHING * (rate - self._odom_rate_hz))
            self._odom_received_at = now
            self._odom = {
                'x': p.x, 'y': p.y, 'yaw': yaw, 'yaw_total': yaw_total,
                'v': msg.twist.twist.linear.x, 'w': msg.twist.twist.angular.z,
            }

    def _on_tick(self):
        """Run one control step: advance the move, publish commands and a snapshot."""
        now = time.monotonic()
        with self._lock:
            self._update_move(now)
            linear, angular = self._cmd
            send_cmd = self._move is not None or self._zero_ticks > 0
            if self._move is None and self._zero_ticks > 0:
                self._zero_ticks -= 1
            payload = self._snapshot(now)

        # Only publish while a test runs (plus a few trailing zeros), so the
        # node does not fight a teleop node or another controller when idle.
        if self._cmd_pub is not None and send_cmd:
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self._cmd_pub.publish(twist)
        if self._fake_pub is not None:
            self._publish_fake_odom(linear, angular)
        self._hub.publish(payload)

    # -- control-loop helpers (executor thread) -----------------------------

    def _update_move(self, now):
        """Start, supervise and finish test moves. Must be called with the lock held.

        A move goes through two phases: 'running' (velocity commanded) and
        'settling' (zero velocity, waiting SETTLE_S before measuring). A
        move stops when it reaches its target (time or odometry, depending
        on stop_on) or is aborted by STOP, odometry loss, a closed browser
        or the closed-loop timeout.

        Args:
            now (float): time.monotonic() of this tick, in seconds.
        """
        move = self._move

        def stop(reason):
            move['phase'] = 'settling'
            move['stopped_at'] = now
            move['elapsed'] = now - move['started_at']
            move['reason'] = reason
            self._cmd = (0.0, 0.0)
            self._zero_ticks = ZERO_REPEAT

        if self._stop_requested:
            self._stop_requested = False
            self._cmd = (0.0, 0.0)
            self._zero_ticks = ZERO_REPEAT
            if move is not None and move['phase'] == 'running':
                stop('stopped by user')

        if move is None and self._pending is not None:
            plan, self._pending = self._pending, None
            self._move = move = {
                'plan': plan, 'start': dict(self._odom), 'started_at': now,
                'phase': 'running', 'progress': 0.0, 'elapsed': 0.0, 'reason': None,
            }
            self._cmd = (plan['linear'], plan['angular'])
        if move is None:
            return

        plan = move['plan']
        forward, _, turned = displacement(move['start'], self._odom)
        move['progress'] = (forward / plan['target_distance'] if plan['target_distance']
                            else turned / plan['target_angle'])

        if move['phase'] == 'running':
            move['elapsed'] = elapsed = now - move['started_at']
            if self._hub.clients == 0:
                stop('browser disconnected')
            elif now - self._odom_received_at > ODOM_TIMEOUT_S:
                stop('odometry stopped arriving')
            elif plan['stop_on'] == 'time':
                if elapsed >= plan['duration']:
                    stop(None)
            elif move['progress'] >= 1.0:
                stop(None)
            elif elapsed > (plan['duration'] * CLOSED_LOOP_TIMEOUT_FACTOR
                            + CLOSED_LOOP_TIMEOUT_EXTRA_S):
                stop('timed out before odometry reached the target')
        elif now - move['stopped_at'] >= SETTLE_S:
            self._result_id += 1
            self._result = summarize_move(
                plan, move['start'], self._odom, move['elapsed'], move['reason'],
                self._wheel_radius, self._wheel_separation)
            self._result['id'] = self._result_id
            self._move = None

    def _snapshot(self, now):
        """Encode the current state for the page. Must be called with the lock held.

        Args:
            now (float): time.monotonic() of this tick, in seconds.

        Returns:
            bytes: UTF-8 JSON with keys t (Unix time, s), odom (pose,
                velocity, rate_hz and age_s, or null before the first
                message), cmd (commanded v m/s, w rad/s), move (running
                test or null), result (last finished test or null) and
                config (node parameters the page needs).
        """
        odom = None
        if self._odom is not None:
            odom = {k: round(self._odom[k], 4) for k in ('x', 'y', 'yaw', 'v', 'w')}
            odom['rate_hz'] = round(self._odom_rate_hz, 1)
            odom['age_s'] = round(now - self._odom_received_at, 2)
        move = None
        if self._move is not None:
            plan = self._move['plan']
            move = {
                'kind': plan['kind'], 'pattern': plan['pattern'], 'stop_on': plan['stop_on'],
                'amount': plan['amount'], 'duration': round(plan['duration'], 2),
                'phase': self._move['phase'], 'elapsed': round(self._move['elapsed'], 2),
                'progress': round(self._move['progress'], 3),
            }
        data = {
            't': round(time.time(), 3),
            'odom': odom,
            'cmd': {'v': round(self._cmd[0], 4), 'w': round(self._cmd[1], 4)},
            'move': move,
            'result': self._result,
            'config': self._config,
        }
        return json.dumps(data, separators=(',', ':')).encode('utf-8')

    def _publish_fake_odom(self, linear, angular):
        """Integrate the commanded velocity one tick and publish it as Odometry.

        Uses midpoint integration of a unicycle model, which is exact enough
        for 50 ms steps. Only used when fake_odom is true.

        Args:
            linear (float): commanded forward speed in m/s.
            angular (float): commanded turn rate in rad/s, counter-clockwise positive.
        """
        x, y, yaw = self._fake_pose
        yaw_mid = yaw + angular * CONTROL_PERIOD_S / 2.0
        x += linear * CONTROL_PERIOD_S * math.cos(yaw_mid)
        y += linear * CONTROL_PERIOD_S * math.sin(yaw_mid)
        yaw = wrap_angle(yaw + angular * CONTROL_PERIOD_S)
        self._fake_pose = [x, y, yaw]

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        self._fake_pub.publish(msg)

    def stop_server(self):
        """Command zero velocity, release browser connections and stop the HTTP server."""
        if self._cmd_pub is not None:
            try:
                self._cmd_pub.publish(Twist())
            except Exception:  # noqa: BLE001 - the ROS context may already be shut down
                pass
        self._hub.close()
        self._httpd.shutdown()
        self._httpd.server_close()


def main(args=None):
    """Run the odom_test_server node until Ctrl+C or ROS shutdown.

    Args:
        args (list[str] | None): command-line arguments for rclpy.init().
    """
    rclpy.init(args=args)
    node = OdomTestServer()
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
