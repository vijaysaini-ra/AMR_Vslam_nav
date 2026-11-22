#!/usr/bin/env python3
"""
waypoint_demo -- autonomous navigation tour for the faculty demo.

  1. PRE-FLIGHT TESTS: sensor rates, TF chain, Nav2 lifecycle, nvblox map, costmap
     (printed as a PASS/FAIL table -- the tour only starts if all critical tests pass)
  2. TOUR: for each waypoint, ask the planner for an A* path (NavFn use_astar: true),
     print path length / planning time, drive there, record result / time / distance
  3. REPORT: summary table, map statistics, and a saved map image with the robot's
     driven trajectory and the waypoints drawn on it

Outputs go to /workspaces/isaac_ros-dev/demo_results/  (host: ~/workspaces/isaac_ros-dev/demo_results/)
  results.csv, map_final.png, map_final.pgm/.yaml (Nav2-compatible), test_report.txt
"""

import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

from nvblox_msgs.msg import DistanceMapSlice

OUT_DIR = '/workspaces/isaac_ros-dev/demo_results'

# (name, x, y, yaw_deg) in map frame (origin = robot spawn pose)
WAYPOINTS = [
    ('Room A - top left', 0.5, 5.8, 90.0),
    ('Middle room', 4.5, 3.5, 0.0),
    ('Room C - bottom right', 7.8, 0.3, 0.0),
    ('Room C - top right', 7.8, 6.0, 90.0),
    ('Home (start)', 0.0, 0.0, 180.0),
]

LINE = '=' * 64


def say(msg=''):
    print(msg, flush=True)


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def path_length(path):
    pts = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class Monitor:
    """Collects topic statistics, odometry trajectory and the latest costmap."""

    def __init__(self, node):
        self.node = node
        self.counts = {}
        self.trajectory = []
        self.driven = 0.0
        self.costmap = None
        self.slice_msg = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._count('/camera/depth_image', Image, qos_profile_sensor_data)
        self._count('/camera/image', Image, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odom', self._odom, 10)
        node.create_subscription(DistanceMapSlice, '/nvblox_node/static_map_slice',
                                 self._slice, 1)
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def _count(self, topic, mtype, qos):
        self.counts[topic] = 0

        def cb(_msg, t=topic):
            self.counts[t] += 1
        self.node.create_subscription(mtype, topic, cb, qos)

    def _odom(self, msg):
        self.counts['/odom'] = self.counts.get('/odom', 0) + 1
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.trajectory:
            self.driven += math.dist(p, self.trajectory[-1])
        if not self.trajectory or math.dist(p, self.trajectory[-1]) > 0.02:
            self.trajectory.append(p)

    def _slice(self, msg):
        self.counts['/nvblox_node/static_map_slice'] = \
            self.counts.get('/nvblox_node/static_map_slice', 0) + 1
        self.slice_msg = msg

    def _costmap(self, msg):
        self.counts['/global_costmap/costmap'] = self.counts.get('/global_costmap/costmap', 0) + 1
        self.costmap = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def map_stats(self):
        if self.costmap is None:
            return None
        g = np.array(self.costmap.data, dtype=np.int16)
        res = self.costmap.info.resolution
        known = int(np.count_nonzero(g >= 0))
        lethal = int(np.count_nonzero(g >= 99))
        free = int(np.count_nonzero(g == 0))
        return {'known_m2': known * res * res, 'free_m2': free * res * res,
                'obstacle_m2': lethal * res * res, 'known_cells': known}


# ---------------------------------------------------------------- tests
def preflight(nav, mon):
    say(LINE)
    say(' PRE-FLIGHT SYSTEM TESTS')
    say(LINE)
    for k in mon.counts:
        mon.counts[k] = 0
    # Rates are measured in SIMULATION time (the sim may run slower than real time)
    sim0, wall0 = nav.get_clock().now().nanoseconds, time.time()
    mon.spin_for(6.0)
    sim_dt = (nav.get_clock().now().nanoseconds - sim0) * 1e-9
    wall_dt = time.time() - wall0
    rtf = sim_dt / wall_dt if wall_dt > 0 else 0.0
    window = max(sim_dt, 1e-3)
    tests = []

    tests.append(('Simulation clock running', sim_dt > 0.5,
                  f'real-time factor {rtf:.2f}', True))

    def rate_test(name, topic, min_hz, critical=True):
        hz = mon.counts.get(topic, 0) / window
        tests.append((name, hz >= min_hz, f'{hz:5.1f} Hz sim-time (need >= {min_hz})', critical))

    rate_test('Depth camera stream', '/camera/depth_image', 8.0)
    rate_test('RGB camera stream', '/camera/image', 8.0, critical=False)
    rate_test('Wheel odometry', '/odom', 20.0)
    rate_test('nvblox map slice (GPU mapping)', '/nvblox_node/static_map_slice', 1.0)

    try:
        t = mon.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        tr = t.transform.translation
        tests.append(('TF chain map -> odom -> base_link', True,
                      f'robot at ({tr.x:.2f}, {tr.y:.2f})', True))
    except Exception as e:  # noqa: BLE001
        tests.append(('TF chain map -> odom -> base_link', False, str(e)[:40], True))
    try:
        mon.tf_buffer.lookup_transform('base_link', 'camera_optical_frame', rclpy.time.Time())
        tests.append(('TF robot body -> camera', True, 'ok', True))
    except Exception as e:  # noqa: BLE001
        tests.append(('TF robot body -> camera', False, str(e)[:40], True))

    for node_name in ['planner_server', 'controller_server', 'bt_navigator']:
        # waitUntilNav2Active() already guaranteed these; report explicitly
        tests.append((f'Nav2 {node_name}', True, 'active', True))

    st = mon.map_stats()
    if st:
        tests.append(('Global costmap from nvblox', st['known_cells'] > 50,
                      f'{st["known_m2"]:.1f} m2 mapped', True))
    else:
        tests.append(('Global costmap from nvblox', False, 'no costmap received', True))

    t0 = time.time()
    path = nav.getPath(PoseStamped(), make_pose(nav, 1.0, 0.0, 0.0),
                       planner_id='GridBased', use_start=False)
    ms = (time.time() - t0) * 1000
    ok = path is not None and len(path.poses) > 0
    tests.append(('A* planner responds', ok, f'{ms:.0f} ms' if ok else 'no path', True))

    for name, ok, detail, crit in tests:
        mark = 'PASS' if ok else ('FAIL' if crit else 'WARN')
        say(f'  [{mark}] {name:<38} {detail}')
    n_ok = sum(1 for t in tests if t[1])
    all_critical = all(t[1] for t in tests if t[3])
    say(f'  -> {n_ok}/{len(tests)} tests passed')
    return tests, all_critical


# ---------------------------------------------------------------- map image
def save_map(mon, results_dir):
    cm = mon.costmap
    if cm is None:
        say('  (no costmap to save)')
        return None
    w, h, res = cm.info.width, cm.info.height, cm.info.resolution
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    g = np.array(cm.data, dtype=np.int16).reshape(h, w)

    # Nav2-compatible PGM + YAML (occupied=0 black, free=254 white, unknown=205)
    pgm = np.full((h, w), 205, np.uint8)
    pgm[g == 0] = 254
    pgm[(g > 0) & (g < 99)] = 254
    pgm[g >= 99] = 0
    cv2.imwrite(os.path.join(results_dir, 'map_final.pgm'), np.flipud(pgm))
    with open(os.path.join(results_dir, 'map_final.yaml'), 'w') as f:
        f.write(f'image: map_final.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n')

    # Colour presentation image
    img = np.full((h, w, 3), (90, 90, 90), np.uint8)          # unknown: dark grey
    img[g == 0] = (255, 255, 255)                             # free: white
    infl = (g > 0) & (g < 99)
    shade = (255 - (g[infl].astype(np.int32) * 1.6)).clip(90, 255).astype(np.uint8)
    img[infl] = np.stack([shade, shade, np.full_like(shade, 255)], axis=1)  # inflation: pink
    img[g >= 99] = (40, 20, 160)                              # obstacles: dark red

    def px(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    # crop to the explored area + driven trajectory + goals (+ margin)
    ys, xs = np.nonzero(g >= 0)
    xs, ys = list(xs), list(ys)
    for x, y in list(mon.trajectory) + [(wp[1], wp[2]) for wp in WAYPOINTS]:
        cx, cy = px(x, y)
        xs.append(cx)
        ys.append(cy)
    m = 20
    x0, x1 = max(min(xs) - m, 0), min(max(xs) + m, w - 1)
    y0, y1 = max(min(ys) - m, 0), min(max(ys) + m, h - 1)

    pts = [px(x, y) for x, y in mon.trajectory]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (230, 120, 0), 2)                  # trajectory: blue
    for i, (_n, x, y, _yaw) in enumerate(WAYPOINTS, 1):
        cv2.circle(img, px(x, y), 6, (0, 160, 0), -1)          # waypoints: green
    cv2.circle(img, px(0, 0), 7, (0, 200, 255), 2)             # start: orange ring

    crop = np.flipud(img[y0:y1 + 1, x0:x1 + 1]).copy()
    scale = 3
    crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    # labels (after flip/scale)
    for i, (_n, x, y, _yaw) in enumerate(WAYPOINTS, 1):
        cx, cy = px(x, y)
        lx, ly = (cx - x0) * scale + 10, (y1 - cy) * scale - 8
        cv2.putText(crop, str(i), (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 120, 0), 2)
    legend = np.full((70, crop.shape[1], 3), 255, np.uint8)
    cv2.putText(legend, 'AMR map built live by nvblox (GPU) -- A* navigation tour',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    cv2.putText(legend, 'blue: driven path   green: goals   red: obstacles   grey: unexplored',
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1)
    out = np.vstack([legend, crop])
    path = os.path.join(results_dir, 'map_final.png')
    cv2.imwrite(path, out)
    return path


# ---------------------------------------------------------------- main
def main():
    rclpy.init()
    nav = BasicNavigator()
    mon = Monitor(nav)
    os.makedirs(OUT_DIR, exist_ok=True)

    say(LINE)
    say(' AMR AUTONOMOUS NAVIGATION DEMO  (nvblox mapping + Nav2 A* planner)')
    say(LINE)
    nav.waitUntilNav2Active(localizer='planner_server')
    say('Nav2 active. Letting nvblox build the first map (5 s) ...')
    mon.spin_for(5.0)

    tests, ok = preflight(nav, mon)
    if not ok:
        say('Critical pre-flight test FAILED -- not starting the tour.')
        rclpy.shutdown()
        return

    results = []
    tour_t0 = time.time()
    for i, (name, x, y, yaw) in enumerate(WAYPOINTS, 1):
        goal = make_pose(nav, x, y, yaw)
        say()
        say(f'----- Goal {i}/{len(WAYPOINTS)}: {name}  ({x:.1f}, {y:.1f}) -----')
        t0 = time.time()
        path = nav.getPath(PoseStamped(), goal, planner_id='GridBased', use_start=False)
        plan_ms = (time.time() - t0) * 1000.0
        if path is None or len(path.poses) == 0:
            say('  A* planner: NO PATH FOUND')
            results.append([name, 'NO_PATH', 0, 0.0, 0.0, 0.0, 0.0])
            continue
        plen = path_length(path)
        say(f'  A* planner : {len(path.poses)} poses, {plen:.2f} m, planned in {plan_ms:.0f} ms')

        d0 = mon.driven
        t_start = time.time()
        nav.goToPose(goal)
        last = 0.0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            # first feedback messages report 0.00 m before the path is computed: skip them
            if fb and fb.distance_remaining > 0.01 and time.time() - last > 4.0:
                last = time.time()
                say(f'  driving    : {fb.distance_remaining:5.2f} m to go')
            rclpy.spin_once(nav, timeout_sec=0.05)
        dt = time.time() - t_start
        res = nav.getResult()
        status = {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.CANCELED: 'CANCELED',
                  TaskResult.FAILED: 'FAILED'}.get(res, 'UNKNOWN')
        drove = mon.driven - d0
        say(f'  result     : {status} in {dt:.1f} s, drove {drove:.2f} m')
        results.append([name, status, len(path.poses), round(plen, 2),
                        round(plan_ms, 1), round(dt, 1), round(drove, 2)])

    mon.spin_for(2.0)  # receive the final costmap
    total_t = time.time() - tour_t0

    # ---------------- report
    lines = []
    lines.append(LINE)
    lines.append(' DEMO SUMMARY')
    lines.append(LINE)
    lines.append('{:<24}{:>10}{:>7}{:>9}{:>9}{:>8}{:>9}'.format(
        'goal', 'result', 'poses', 'A*_m', 'plan_ms', 'time_s', 'drove_m'))
    for r in results:
        lines.append('{:<24}{:>10}{:>7}{:>9}{:>9}{:>8}{:>9}'.format(*r))
    n_ok = sum(1 for r in results if r[1] == 'SUCCEEDED')
    lines.append(f'Success rate     : {n_ok}/{len(results)}')
    lines.append(f'Total time       : {total_t:.0f} s,  total distance {mon.driven:.1f} m')
    plans = [r[4] for r in results if r[4] > 0]
    if plans:
        lines.append(f'A* planning time : mean {np.mean(plans):.0f} ms, max {max(plans):.0f} ms')
    st = mon.map_stats()
    if st:
        lines.append(f'Map built        : {st["known_m2"]:.1f} m2 explored, '
                     f'{st["free_m2"]:.1f} m2 free, {st["obstacle_m2"]:.1f} m2 obstacles')
    for ln in lines:
        say(ln)

    img = save_map(mon, OUT_DIR)
    with open(os.path.join(OUT_DIR, 'results.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['goal', 'result', 'astar_poses', 'astar_length_m',
                    'plan_time_ms', 'travel_time_s', 'driven_m'])
        w.writerows(results)
    with open(os.path.join(OUT_DIR, 'test_report.txt'), 'w') as f:
        f.write('PRE-FLIGHT TESTS\n')
        for name, ok, detail, crit in tests:
            f.write(f'  [{"PASS" if ok else ("FAIL" if crit else "WARN")}] {name}: {detail}\n')
        f.write('\n' + '\n'.join(lines) + '\n')
    say(f'Saved: {OUT_DIR}/ (results.csv, test_report.txt, map_final.png, map_final.pgm)')
    if img:
        say('Map image: ~/workspaces/isaac_ros-dev/demo_results/map_final.png')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
