import math
import os
import random
import sys
import time

import mujoco

MODEL_PATH = "cafe_building.xml"

MODE_MANUAL = "manual"
MODE_RANDOM = "random"
MODE_AREA = "area"
MODE_SPOT = "spot"
MODE_RETURN = "return"
MODE_DOCKED = "docked"
MODE_STUCK = "stuck"
MODE_ESTOP = "estop"

LIDAR_MAX_RANGE = 10.0
SAFE_DISTANCE = 0.10
REACTION_HZ = 20.0

MAP_SIZE = 20.0
MAP_RES = 0.05
GRID_W = int(MAP_SIZE / MAP_RES)
GRID_H = int(MAP_SIZE / MAP_RES)

DEFAULT_DOCK_POS = (-4.3, -4.5)

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

class Battery:
    def __init__(self, capacity=1.0, low=0.20, critical=0.08):
        self.capacity = float(capacity)
        self.level = float(capacity)
        self.low = float(low)
        self.critical = float(critical)
        self.is_charging = False

    def update(self, dt, activity=0.0):
        if self.is_charging:
            self.level = min(self.capacity, self.level + dt * 0.02)
            return
        base = 0.0025
        self.level = max(0.0, self.level - dt * (base + 0.01 * abs(activity)))

    def low_battery(self):
        return self.level <= self.low

    def critical_battery(self):
        return self.level <= self.critical

class OccupancyGrid:
    def __init__(self):
        self.origin_x = -MAP_SIZE / 2
        self.origin_y = -MAP_SIZE / 2
        self.logodds = [[0.0 for _ in range(GRID_H)] for _ in range(GRID_W)]
        self.visited = [[0 for _ in range(GRID_H)] for _ in range(GRID_W)]

    def _to_grid(self, x, y):
        gx = int((x - self.origin_x) / MAP_RES)
        gy = int((y - self.origin_y) / MAP_RES)
        return gx, gy

    def mark_visited(self, x, y):
        gx, gy = self._to_grid(x, y)
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            self.visited[gx][gy] = 1

    def update_ray(self, x, y, theta, dist):
        gx0, gy0 = self._to_grid(x, y)
        if not (0 <= gx0 < GRID_W and 0 <= gy0 < GRID_H):
            return

        dist = max(0.0, min(LIDAR_MAX_RANGE, float(dist)))
        hit = dist < (LIDAR_MAX_RANGE * 0.95)
        end_dist = max(0.0, dist - SAFE_DISTANCE)
        steps = int(end_dist / MAP_RES)
        ct = math.cos(theta)
        st = math.sin(theta)

        for i in range(1, steps + 1):
            px = x + ct * (i * MAP_RES)
            py = y + st * (i * MAP_RES)
            gx, gy = self._to_grid(px, py)
            if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                self.logodds[gx][gy] = max(-3.0, self.logodds[gx][gy] - 0.06)

        if hit and dist > 0.0:
            px = x + ct * dist
            py = y + st * dist
            gx, gy = self._to_grid(px, py)
            if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                self.logodds[gx][gy] = min(3.0, self.logodds[gx][gy] + 0.18)

    def coverage_ratio(self):
        visited = sum(sum(row) for row in self.visited)
        total = GRID_W * GRID_H
        return visited / total if total else 0.0

class VacuumAutonomy:
    def __init__(self, model, data, dock_xy=DEFAULT_DOCK_POS):
        self.model = model
        self.data = data
        self.mode = MODE_RANDOM
        self.dock_xy = (float(dock_xy[0]), float(dock_xy[1]))
        self.grid = OccupancyGrid()
        self.battery = Battery()

        self._init_ids()
        self._last_pose = None
        self._last_progress_t = 0.0
        self._stuck_events = 0
        self._recovery_t = 0.0
        self._spot_t = 0.0
        self._random_heading = 0.0
        self._manual_cmd = (0.0, 0.0)

    def _init_ids(self):
        try:
            self.act_fl = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_fl")
            self.act_fr = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_fr")
            self.act_bl = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_bl")
            self.act_br = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_br")

            self.s_lidar_ids = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_0"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_l30"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_r30"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_l90"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_r90"),
            ]
            self.s_cliff_ids = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "cliff_f"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "cliff_l"),
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "cliff_r"),
            ]
            self.s_bumper = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "bumper_touch")
            self.body_robot = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        except Exception as e:
            print(f"初始化 ID 失败: {e}")
            sys.exit(1)

    def set_manual(self, left, right):
        self._manual_cmd = (float(left), float(right))
        self.mode = MODE_MANUAL

    def set_mode(self, mode):
        if mode not in {MODE_MANUAL, MODE_RANDOM, MODE_AREA, MODE_SPOT, MODE_RETURN, MODE_DOCKED, MODE_STUCK, MODE_ESTOP}:
            return
        self.mode = mode

    def _get_pose(self):
        pos = self.data.body_xpos[self.body_robot]
        quat = self.data.body_xquat[self.body_robot]
        w, x, y, z = quat
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), float(yaw)

    def _read_range(self, sid):
        v = float(self.data.sensordata[self.model.sensor_adr[sid]])
        if v < 0:
            return LIDAR_MAX_RANGE
        return min(LIDAR_MAX_RANGE, v)

    def _sense(self):
        lidar = [self._read_range(i) for i in self.s_lidar_ids]
        cliff = [self._read_range(i) for i in self.s_cliff_ids]
        is_cliff = any(d > 0.05 for d in cliff)
        bump = float(self.data.sensordata[self.model.sensor_adr[self.s_bumper]]) > 0.0
        return lidar, cliff, is_cliff, bump

    def _apply_ctrl(self, left, right):
        left = _clamp(left, -1.0, 1.0)
        right = _clamp(right, -1.0, 1.0)
        self.data.ctrl[self.act_fl] = left
        self.data.ctrl[self.act_bl] = left
        self.data.ctrl[self.act_fr] = right
        self.data.ctrl[self.act_br] = right

    def _safety_filter(self, left, right, lidar, is_cliff, bump):
        if os.path.exists("ESTOP"):
            return 0.0, 0.0, MODE_ESTOP

        if is_cliff:
            return -0.25, -0.25, MODE_STUCK

        if bump:
            return -0.35, -0.35, MODE_STUCK

        front = lidar[0]
        if front < SAFE_DISTANCE:
            return -0.25, -0.25, MODE_STUCK

        slow = 1.0
        if front < 0.35:
            slow = max(0.15, (front - SAFE_DISTANCE) / (0.35 - SAFE_DISTANCE))
        if min(lidar[1], lidar[2]) < 0.25:
            slow = min(slow, 0.4)
        return left * slow, right * slow, None

    def _drive_to(self, x, y, yaw, goal_x, goal_y):
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)
        target = math.atan2(dy, dx)
        diff = _wrap_pi(target - yaw)
        if dist < 0.35:
            return 0.0, 0.0, True
        if abs(diff) > 0.35:
            t = 0.6 if diff > 0 else -0.6
            return -t, t, False
        fwd = 0.6
        steer = _clamp(diff * 0.8, -0.25, 0.25)
        return fwd - steer, fwd + steer, False

    def _random_policy(self, x, y, yaw, lidar):
        if random.random() < 0.02:
            self._random_heading = _wrap_pi(self._random_heading + random.uniform(-0.8, 0.8))
        avoid_bias = 0.0
        avoid_bias += (0.55 - lidar[1]) if lidar[1] < 0.55 else 0.0
        avoid_bias -= (0.55 - lidar[2]) if lidar[2] < 0.55 else 0.0
        avoid_bias += (0.30 - lidar[3]) * 1.2 if lidar[3] < 0.30 else 0.0
        avoid_bias -= (0.30 - lidar[4]) * 1.2 if lidar[4] < 0.30 else 0.0
        steer = _clamp(self._random_heading * 0.25 + avoid_bias * 0.9, -0.6, 0.6)
        fwd = 0.55 if lidar[0] > 0.5 else 0.2
        return fwd - steer, fwd + steer

    def _spot_policy(self, dt, lidar):
        self._spot_t += dt
        t = self._spot_t
        if t < 3.0:
            return 0.4, 0.4
        if t < 12.0:
            w = 0.35 + 0.03 * (t - 3.0)
            return 0.45 + w, 0.45 - w
        self._spot_t = 0.0
        return 0.0, 0.0

    def _area_policy(self, x, y, yaw, lidar):
        step = 0.55
        band = int((y - (-2.0)) / step)
        target_y = (-2.0) + band * step
        dir_sign = 1.0 if (band % 2 == 0) else -1.0
        target_x = 4.0 if dir_sign > 0 else -4.0
        if abs(y - target_y) > 0.25:
            goal_x, goal_y = x, target_y
        else:
            goal_x, goal_y = target_x, target_y
        left, right, done = self._drive_to(x, y, yaw, goal_x, goal_y)
        if done and abs(y - target_y) <= 0.25:
            goal_x, goal_y = x, target_y + step
            left, right, _ = self._drive_to(x, y, yaw, goal_x, goal_y)
        return left, right

    def _stuck_recovery(self, dt):
        self._recovery_t += dt
        if self._recovery_t < 1.0:
            return -0.6, -0.6, False
        if self._recovery_t < 2.5:
            turn = 0.8 if (self._stuck_events % 2 == 0) else -0.8
            return -turn, turn, False
        self._recovery_t = 0.0
        self._stuck_events += 1
        if self._stuck_events >= 5:
            self.mode = MODE_ESTOP
            return 0.0, 0.0, True
        return 0.0, 0.0, True

    def step(self, dt):
        mode_path = "VACUUM_MODE"
        if os.path.exists(mode_path):
            try:
                desired = open(mode_path, "r", encoding="utf-8").read().strip().lower()
                if desired:
                    self.set_mode(desired)
            except Exception:
                pass
        if self.mode == MODE_MANUAL:
            manual_path = "MANUAL"
            if os.path.exists(manual_path):
                try:
                    txt = open(manual_path, "r", encoding="utf-8").read().strip()
                    parts = txt.replace(",", " ").split()
                    if len(parts) >= 2:
                        self._manual_cmd = (float(parts[0]), float(parts[1]))
                except Exception:
                    pass

        x, y, yaw = self._get_pose()
        lidar, _, is_cliff, bump = self._sense()

        self.grid.mark_visited(x, y)
        angles = [0.0, math.radians(30), math.radians(-30), math.radians(90), math.radians(-90)]
        for ang, dist in zip(angles, lidar):
            self.grid.update_ray(x, y, yaw + ang, dist)

        activity = 0.0
        if self.mode in {MODE_RANDOM, MODE_AREA, MODE_SPOT, MODE_RETURN}:
            activity = 1.0
        self.battery.update(dt, activity=activity)

        if self.battery.critical_battery():
            self.mode = MODE_RETURN
        elif self.battery.low_battery() and self.mode not in {MODE_RETURN, MODE_DOCKED, MODE_ESTOP}:
            self.mode = MODE_RETURN

        if self.mode == MODE_ESTOP:
            self._apply_ctrl(0.0, 0.0)
            return

        if self._last_pose is None:
            self._last_pose = (x, y)
            self._last_progress_t = 0.0
        else:
            dx = x - self._last_pose[0]
            dy = y - self._last_pose[1]
            prog = math.hypot(dx, dy)
            if prog > 0.03:
                self._last_pose = (x, y)
                self._last_progress_t = 0.0
            else:
                self._last_progress_t += dt

        if self._last_progress_t > 1.2 and self.mode not in {MODE_STUCK, MODE_ESTOP, MODE_DOCKED}:
            self.mode = MODE_STUCK

        if self.mode == MODE_STUCK:
            left, right, done = self._stuck_recovery(dt)
            lf, rf, force = self._safety_filter(left, right, lidar, is_cliff, bump)
            if force == MODE_ESTOP:
                self.mode = MODE_ESTOP
            self._apply_ctrl(lf, rf)
            if done and self.mode == MODE_STUCK:
                self.mode = MODE_RANDOM
            return

        if self.mode == MODE_DOCKED:
            self.battery.is_charging = True
            self._apply_ctrl(0.0, 0.0)
            if self.battery.level >= self.battery.capacity * 0.95:
                self.battery.is_charging = False
                self.mode = MODE_RANDOM
            return

        left = 0.0
        right = 0.0
        if self.mode == MODE_MANUAL:
            left, right = self._manual_cmd
        elif self.mode == MODE_RANDOM:
            left, right = self._random_policy(x, y, yaw, lidar)
        elif self.mode == MODE_AREA:
            left, right = self._area_policy(x, y, yaw, lidar)
        elif self.mode == MODE_SPOT:
            left, right = self._spot_policy(dt, lidar)
        elif self.mode == MODE_RETURN:
            left, right, done = self._drive_to(x, y, yaw, self.dock_xy[0], self.dock_xy[1])
            if done:
                self.mode = MODE_DOCKED

        lf, rf, force_mode = self._safety_filter(left, right, lidar, is_cliff, bump)
        if force_mode == MODE_STUCK:
            self.mode = MODE_STUCK
        elif force_mode == MODE_ESTOP:
            self.mode = MODE_ESTOP
            lf, rf = 0.0, 0.0

        self._apply_ctrl(lf, rf)

def main():
    mode = MODE_RANDOM
    dock_xy = DEFAULT_DOCK_POS
    if "--mode" in sys.argv:
        try:
            mode = sys.argv[sys.argv.index("--mode") + 1].strip().lower()
        except Exception:
            pass
    if "--dock" in sys.argv:
        try:
            dock_s = sys.argv[sys.argv.index("--dock") + 1].strip()
            a, b = dock_s.replace(",", " ").split()[:2]
            dock_xy = (float(a), float(b))
        except Exception:
            pass

    print(f"加载模型: {MODEL_PATH}")
    try:
        model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Error: {e}")
        return

    autonomy = VacuumAutonomy(model, data, dock_xy=dock_xy)
    autonomy.set_mode(mode)

    viewer = None
    try:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    except:
        pass

    dt = float(model.opt.timestep)
    control_dt = 1.0 / REACTION_HZ
    acc = 0.0
    last = time.time()

    print("系统启动。模式: random / area / spot / return / manual / estop")
    while True:
        if viewer and not viewer.is_running(): break

        now = time.time()
        wall_dt = now - last
        last = now
        acc += wall_dt

        while acc >= dt:
            mujoco.mj_step(model, data)
            acc -= dt

        autonomy_timer = getattr(main, "_autonomy_timer", 0.0) + wall_dt
        setattr(main, "_autonomy_timer", autonomy_timer)
        if autonomy_timer >= control_dt:
            setattr(main, "_autonomy_timer", 0.0)
            autonomy.step(control_dt)

        if viewer:
            viewer.sync()

        time.sleep(0.001)

if __name__ == "__main__":
    main()
