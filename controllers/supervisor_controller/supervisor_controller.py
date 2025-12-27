# supervisor_controller.py
# Non-blocking Supervisor controller (includes random ball placement + simplified absorption logic)
from controller import Supervisor
import math
import numpy as np
import random
import sys

# ==== Initialization ====
DEFAULT_VELOCITY = 0.3 # m/s
DEFAULT_ANGULAR_VELOCITY = 3  # rad/s
supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())
dt = TIME_STEP / 1000.0  # seconds

SCORE = 0
STEEL_STORED = 0
PING_STORED = 0
STEEL_HIT = 0
PING_HIT = 0

# Ensure the world contains DEF MY_ROBOT
robot = supervisor.getFromDef("MY_ROBOT")
if robot is None:
    print("ERROR: DEF MY_ROBOT not found in world.")
    sys.exit(1)

trans = robot.getField("translation")
rot_field = robot.getField("rotation")

# ==== Utility functions ====
def _normalize_angle(a):
    """Normalize angle to [-pi, pi]"""
    return (a + math.pi) % (2 * math.pi) - math.pi

# ==== Non-blocking MotionController (replacement for blocking move_to) ====
class MotionController:
    def __init__(self, trans_field, rot_field, dt):
        self.trans = trans_field
        self.rot = rot_field
        self.dt = dt
        self.active = False
        self.phase = None
        self.start_pos = None
        self.target_pos = None
        self.start_angle = 0.0
        self.target_angle = None
        self.velocity = 0.3
        self.angular_speed = DEFAULT_ANGULAR_VELOCITY  # rad/s (default 90 deg/s)
        self.direction = np.array([0.0, 0.0, 0.0])
        self.total_dist = 0.0
        self.traveled = 0.0

    def start(self, x, y, velocity=None, angle=None):
        """Start a motion task (non-blocking)"""
        cur_pos = np.array(self.trans.getSFVec3f(), dtype=float)
        cur_angle = _normalize_angle(self.rot.getSFRotation()[3])

        target = np.array([x, y, cur_pos[2]], dtype=float)
        delta = target - cur_pos
        dist = np.linalg.norm(delta)

        self.start_pos = cur_pos
        self.target_pos = target
        self.start_angle = cur_angle
        self.target_angle = _normalize_angle(angle) if angle is not None else None
        self.velocity = DEFAULT_VELOCITY if velocity is None else velocity
        self.total_dist = dist
        self.traveled = 0.0
        self.direction = (delta / dist) if dist > 1e-9 else np.array([0.0, 0.0, 0.0])

        if dist <= 1e-6:
            if self.target_angle is None:
                self.phase = None
                self.active = False
                return
            else:
                self.phase = 'rotate_only'
                self.active = True
                return

        if self.target_angle is None:
            # Rotate in place to face the target, then move in a straight line (do not change orientation)
            target_yaw = math.atan2((y - cur_pos[1]), (x - cur_pos[0]))
            self.target_angle = _normalize_angle(target_yaw)
            self.phase = 'rotate_then_move'
        else:
            # Move and interpolate rotation to the target angle simultaneously
            self.phase = 'move_and_rotate'

        self.active = True

    def update(self):
        """Called each frame to advance motion; returns True if the task is completed or idle"""
        if not self.active:
            return True

        cur_pos = np.array(self.trans.getSFVec3f(), dtype=float)
        step_dist = self.velocity * self.dt

        # helper: single-step rotate towards target_ang
        def step_rotate_towards(target_ang):
            cur = _normalize_angle(self.rot.getSFRotation()[3])
            rem = _normalize_angle(target_ang - cur)
            if abs(rem) <= 1e-3:
                self.rot.setSFRotation([0, 0, 1, target_ang])
                return True
            max_step = self.angular_speed * self.dt
            step_ang = max_step if abs(rem) > max_step else abs(rem)
            sign = 1.0 if rem > 0 else -1.0
            self.rot.setSFRotation([0, 0, 1, _normalize_angle(cur + sign * step_ang)])
            return False

        if self.phase == 'rotate_only':
            done = step_rotate_towards(self.target_angle)
            if done:
                self.active = False
                self.phase = None
                return True
            return False

        if self.phase == 'rotate_then_move':
            rotated = step_rotate_towards(self.target_angle)
            if not rotated:
                return False
            remaining = np.linalg.norm(self.target_pos - cur_pos)
            if remaining <= step_dist:
                self.trans.setSFVec3f(self.target_pos.tolist())
                self.active = False
                self.phase = None
                return True
            new_pos = cur_pos + self.direction * step_dist
            self.trans.setSFVec3f(new_pos.tolist())
            return False

        if self.phase == 'move_and_rotate':
            remaining = np.linalg.norm(self.target_pos - cur_pos)
            if remaining <= step_dist:
                self.trans.setSFVec3f(self.target_pos.tolist())
                self.rot.setSFRotation([0, 0, 1, self.target_angle])
                self.active = False
                self.phase = None
                return True
            # Position update
            new_pos = cur_pos + self.direction * step_dist
            self.trans.setSFVec3f(new_pos.tolist())
            # Angle interpolated based on travelled/total_dist
            self.traveled += step_dist
            progress = min(self.traveled / (self.total_dist + 1e-9), 1.0)
            current_angle = _normalize_angle(self.start_angle + _normalize_angle(self.target_angle - self.start_angle) * progress)
            self.rot.setSFRotation([0,0,1,current_angle])
            return False

        return True

    def cancel(self):
        self.active = False
        self.phase = None

# ==== Randomize balls (start) ====
BALL_PREFIX = "BALL_"
BALL_COUNT = 40
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86
BALL_RADIUS = 0.02
MIN_SEPARATION = 2.0 * BALL_RADIUS + 0.001
MAX_TRIES_PER_BALL = 2000
SETTLE_STEPS_AFTER_PLACEMENT = max(1, int(0.2 / dt))
Z_EPS = 0.001

# Robot occupancy parameters (avoid when placing balls at start)
ROBOT_X = -0.8
ROBOT_Y = 0.0
ROBOT_HALF_SIZE = 0.1
CLEARANCE = 0.05

def _point_in_rect(px, py, rect):
    return (rect[0] <= px <= rect[1]) and (rect[2] <= py <= rect[3])

def randomize_balls(seed=None, ensure_no_overlap=True):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    avoid_zones = []
    margin = ROBOT_HALF_SIZE + CLEARANCE + BALL_RADIUS
    robot_rect = (ROBOT_X - margin, ROBOT_X + margin, ROBOT_Y - margin, ROBOT_Y + margin)
    avoid_zones.append(robot_rect)

    placed_positions = []

    for i in range(BALL_COUNT):
        name = f"{BALL_PREFIX}{i}"
        node = supervisor.getFromDef(name)
        if node is None:
            print(f"Warning: {name} not found, skipping.")
            continue
        trans_field = node.getField("translation")

        if not ensure_no_overlap:
            for attempt in range(MAX_TRIES_PER_BALL):
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
                blocked = any(_point_in_rect(x,y,rect) for rect in avoid_zones)
                if not blocked:
                    break
            else:
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
            z = BALL_RADIUS + Z_EPS
            trans_field.setSFVec3f([x,y,z])
            node.resetPhysics()
            placed_positions.append((x,y))
            continue

        success = False
        for attempt in range(MAX_TRIES_PER_BALL):
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)
            if any(_point_in_rect(x,y,rect) for rect in avoid_zones):
                continue
            ok = True
            for (px,py) in placed_positions:
                if (x-px)**2 + (y-py)**2 < (MIN_SEPARATION**2):
                    ok = False
                    break
            if ok:
                z = BALL_RADIUS + Z_EPS
                trans_field.setSFVec3f([x,y,z])
                node.resetPhysics()
                placed_positions.append((x,y))
                success = True
                break
        if not success:
            for attempt in range(MAX_TRIES_PER_BALL):
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
                if any(_point_in_rect(x,y,rect) for rect in avoid_zones):
                    continue
                z = BALL_RADIUS + Z_EPS
                trans_field.setSFVec3f([x,y,z])
                node.resetPhysics()
                placed_positions.append((x,y))
                success = True
                break
        if not success:
            print(f"Warning: failed to place {name} without overlap/avoid zone. Forcing placement.")
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)
            z = BALL_RADIUS + Z_EPS
            trans_field.setSFVec3f([x,y,z])
            node.resetPhysics()
            placed_positions.append((x,y))

    # Let physics settle for a few steps (short blocking)
    for _ in range(SETTLE_STEPS_AFTER_PLACEMENT):
        supervisor.step(TIME_STEP)

    print(f"Randomized {len(placed_positions)} balls. seed={seed}, no_overlap={ensure_no_overlap}")

# ==== Simplified absorption monitoring (per-frame simple box check) ====
ABSORB_BOX_HALF_X = 0.12   # robot-local x half-width
ABSORB_BOX_HALF_Y = 0.05   # robot-local y half-width
ABSORB_DISTANCE = 0  # robot-local absorption distance threshold
ABSORB_LOCATION = (-1.1, 0.0, 0.4)

# Only record whether absorbed to avoid duplicate absorption
ball_simple_state = {}  # name -> {"absorbed": bool}

def monitor_simple_init(ball_prefix="BALL_", ball_count=40):
    """Initialize ball_simple_state (call once)"""
    ball_simple_state.clear()
    for i in range(ball_count):
        name = f"{ball_prefix}{i}"
        node = supervisor.getFromDef(name)
        if node is None:
            continue
        ball_simple_state[name] = {"absorbed": False}
    # print(f"[monitor_simple] initialized for {len(ball_simple_state)} balls")

def monitor_simple_step(ball_prefix="BALL_", ball_count=40, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION):
    """
    Called each frame: if a ball enters the robot-local rectangular area it is immediately absorbed (teleport + resetPhysics).
    The region is based on the robot's current position (relative to the robot).

    Type detection: make a single attempt to read the proto/name (getProtoName),
    if the proto name contains "steel" it is considered 'steel', otherwise 'ping' by default.
    """
    global PING_STORED, STEEL_STORED
    # Read robot current position (world coordinates)
    robot_pos = np.array(robot.getField("translation").getSFVec3f(), dtype=float)
    rx, ry = float(robot_pos[0]), float(robot_pos[1])
    # Use the original rotation handling (assumes rotation around z axis based on angle)
    robor_rot = np.array(robot.getField("rotation").getSFRotation())
    rangle = float(robor_rot[3])

    min_distance = 2

    for i in range(ball_count):
        name = f"{ball_prefix}{i}"
        # Ensure the ball exists and is in the state table
        if name not in ball_simple_state:
            node_tmp = supervisor.getFromDef(name)
            if node_tmp is None:
                continue
            ball_simple_state[name] = {"absorbed": False}

        if ball_simple_state[name]["absorbed"]:
            continue

        node = supervisor.getFromDef(name)
        if node is None:
            continue

        # Read position
        pos = node.getField("translation").getSFVec3f()
        bx, by = float(pos[0]), float(pos[1])
        temp_distance = math.sqrt((bx - rx)**2 + (by - ry)**2)
        min_distance = min(min_distance, temp_distance)

        # World -> robot coordinate transform (2D, assuming rotation around z axis)
        x_rel = bx - rx
        y_rel = by - ry
        x_ball_robot = x_rel * math.cos(-rangle) - y_rel * math.sin(-rangle)
        y_ball_robot = x_rel * math.sin(-rangle) + y_rel * math.cos(-rangle)

        # Single attempt: use Robot.name to determine type, default 'ping'
        name_field = node.getField("robotName").getSFString()
        ball_type = "ping"
        if name_field is not None:
            node_name = name_field.lower()
            if "steel" in node_name:
                ball_type = "steel"
            else:
                ball_type = "ping"

        # Decide absorption based on type
        absorbed = False
        if ball_type == "ping":
            if (x_ball_robot > 0) and (x_ball_robot < half_x) and (abs(y_ball_robot) < half_y):
                absorbed = True
                PING_STORED += 1
        elif ball_type == "steel":
            # Per your logic: x_ball_robot > 0 and (x_ball_robot - 0.2 < half_x)
            if (x_ball_robot > 0) and (x_ball_robot < half_x - 0.01) and (abs(y_ball_robot) < half_y):
                absorbed = True
                STEEL_STORED += 1

        if absorbed:
            # Absorption action: teleport and resetPhysics
            node.getField("translation").setSFVec3f([absorb_location[0], absorb_location[1], absorb_location[2]])
            try:
                node.resetPhysics()
            except Exception:
                pass
            ball_simple_state[name]["absorbed"] = True
            SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1
            # print(f"[monitor_simple] absorbed {name} type={ball_type} at x={x_ball_robot:.3f}, y={y_ball_robot:.3f}, distance={temp_distance:.3f} -> moved to {absorb_location}")
            print(f"Score: {SCORE} | Ping Hit: {PING_HIT} | Steel Hit: {STEEL_HIT} | Steel Stored: {STEEL_STORED} | Ping Stored: {PING_STORED}")


    # print(f"[monitor_simple] min_distance to balls: {min_distance:.3f}")

# SmoothRandomObstacle: 平滑移动版（匀速直线 + 匀速旋转）
import random, math, numpy as np

class SmoothRandomObstacle:
    def __init__(self, supervisor, def_name="OTHER1", interval=3.0, points=None, z=0.01,
                 xlim=0.85, ylim=0.85, seed=None, default_velocity=1, max_angular_speed=5.0):
        """
        supervisor: Supervisor 实例
        def_name: WORLD 中 OTHER1 的 DEF 名称
        interval: 到达一个点后停留多少秒（以前为瞬移间隔）
        points: 可选 list of (x,y,yaw)
        z: 固定 z 高度
        default_velocity: 线速度 m/s（匀速直线移动）
        max_angular_speed: 角速度上限 rad/s（安全限制）
        """
        self.supervisor = supervisor
        self.def_name = def_name
        self.interval = float(interval)
        self.z = float(z)
        self.xlim = float(xlim)
        self.ylim = float(ylim)
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        # node & fields cache
        self.node = supervisor.getFromDef(def_name)
        if self.node is None:
            print(f"[SmoothRandomObstacle] WARNING: DEF {def_name} not found")
            self.trans_field = None
            self.rot_field = None
        else:
            try:
                self.trans_field = self.node.getField("translation")
                self.rot_field = self.node.getField("rotation")
            except Exception:
                self.trans_field = None
                self.rot_field = None

        # points
        if points is None:
            self.points = self._generate_random_points(12)
        else:
            pts = []
            for p in points:
                if len(p) >= 3:
                    pts.append((float(p[0]), float(p[1]), float(p[2])))
                else:
                    pts.append((float(p[0]), float(p[1]), random.uniform(-math.pi, math.pi)))
            self.points = pts

        # motion params
        self.idx = 0
        self.enabled = True
        self.state = "idle"   # idle, moving, arrived
        self.default_velocity = float(default_velocity)
        self.max_angular_speed = float(max_angular_speed)

        # dynamic motion state
        self.target = None         # (x,y,yaw)
        self.start_pos = None      # numpy array [x,y,z]
        self.start_yaw = 0.0
        self.move_dir = None       # unit vec in XY
        self.total_dist = 0.0
        self.traveled = 0.0
        self.angular_speed = 0.0   # signed rad/s
        self.arrival_time = None
        self.last_action_time = self.supervisor.getTime()

    def _generate_random_points(self, n):
        pts = []
        for _ in range(n):
            x = random.uniform(-self.xlim, self.xlim)
            y = random.uniform(-self.ylim, self.ylim)
            yaw = random.uniform(-math.pi, math.pi)
            pts.append((x, y, yaw))
        return pts

    def normalize_angle(self, a):
        return (a + math.pi) % (2*math.pi) - math.pi

    def _get_current_pose(self):
        """返回当前机器人节点的世界位姿 (x,y,z,yaw)"""
        if self.node is None:
            return None
        try:
            pos = self.trans_field.getSFVec3f()
            rot = self.rot_field.getSFRotation()
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            yaw = float(rot[3]) if abs(rot[0])<1e-6 and abs(rot[1])<1e-6 else float(rot[3])
            # 注意：这里假定 rotation axis 为 z 方向（常见），若不是会偏差
            return (x, y, z, yaw)
        except Exception:
            return None

    def start(self, immediate_move=True):
        """启用并（可选）立刻开始移动到当前 idx"""
        self.enabled = True
        if immediate_move:
            self._prepare_move_to_idx(self.idx)
            # do not set last_action_time here; update() 会推进
        else:
            # idle until interval passes
            self.state = "arrived"
            self.arrival_time = self.supervisor.getTime()
            self.last_action_time = self.arrival_time

    def stop(self):
        self.enabled = False

    def _prepare_move_to_idx(self, idx):
        """准备从当前位姿出发去到 self.points[idx]，计算速度与角速度"""
        if self.node is None:
            return
        cur = self._get_current_pose()
        if cur is None:
            return
        cur_x, cur_y, cur_z, cur_yaw = cur
        tx, ty, tyaw = self.points[idx % len(self.points)]
        # start / target
        self.start_pos = np.array([cur_x, cur_y, cur_z], dtype=float)
        self.start_yaw = self.normalize_angle(cur_yaw)
        self.target = (float(tx), float(ty), float(self.normalize_angle(tyaw)))
        # vector & distance
        vec = np.array([tx - cur_x, ty - cur_y], dtype=float)
        dist = float(np.linalg.norm(vec))
        if dist > 1e-9:
            self.move_dir = vec / dist
        else:
            self.move_dir = np.array([0.0, 0.0], dtype=float)
        self.total_dist = dist
        self.traveled = 0.0
        # compute travel_time based on default_velocity (若速度为0则瞬移)
        if self.default_velocity <= 0 or dist < 1e-9:
            travel_time = 0.0
        else:
            travel_time = dist / self.default_velocity
        # compute angular diff and angular_speed so rotation finishes at same time as translation
        angle_diff = self.normalize_angle(self.target[2] - self.start_yaw)
        if travel_time > 1e-9:
            needed_ang_speed = angle_diff / travel_time
        else:
            # if no translation time, rotate at capped max speed to reach quickly
            # choose sign of angle_diff
            needed_ang_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angle_diff))
        # cap angular speed to max_angular_speed (preserve sign)
        if abs(needed_ang_speed) > self.max_angular_speed:
            needed_ang_speed = math.copysign(self.max_angular_speed, needed_ang_speed)
        self.angular_speed = float(needed_ang_speed)
        # set state
        self.state = "moving"
        # reset last_action_time
        self.last_action_time = self.supervisor.getTime()
        # Note: we do NOT immediately write to fields here; update() will step them per-frame

    def _finalize_arrival(self):
        """在到达位置后的处理：把节点固定到目标位姿，并 resetPhysics，然后进入 arrived 状态"""
        if self.node is None or self.target is None:
            self.state = "idle"
            return
        tx, ty, tyaw = self.target
        try:
            # set final exact pose
            if self.trans_field is not None:
                self.trans_field.setSFVec3f([tx, ty, float(self.z)])
            if self.rot_field is not None:
                self.rot_field.setSFRotation([0.0, 0.0, 1.0, float(tyaw)])
            try:
                self.node.resetPhysics()
            except Exception:
                pass
        except Exception:
            pass
        # advance index for next movement after dwell
        self.idx = (self.idx + 1) % len(self.points)
        self.state = "arrived"
        self.arrival_time = self.supervisor.getTime()
        self.last_action_time = self.arrival_time

    def update(self):
        """
        每帧调用：推进平滑移动/旋转或在到达后等待 interval 然后准备下一个移动
        """
        if not self.enabled or self.node is None:
            return
        # compute dt from TIME_STEP (assumes TIME_STEP 全局存在)
        try:
            dt = float(TIME_STEP) / 1000.0
        except Exception:
            # fallback: use supervisor time difference
            dt = 0.05

        now = self.supervisor.getTime()

        if self.state == "idle":
            # 启动一次移动
            self._prepare_move_to_idx(self.idx)
            return

        if self.state == "arrived":
            # waiting period before next move
            if (now - self.arrival_time) >= self.interval:
                # prepare next move
                self._prepare_move_to_idx(self.idx)
            return

        if self.state == "moving":
            # get current pose (may have been changed externally), compute incremental update
            try:
                # current position & yaw
                cur_pos = np.array(self.trans_field.getSFVec3f(), dtype=float)
                cur_x, cur_y = float(cur_pos[0]), float(cur_pos[1])
                # current yaw
                cur_rot = self.rot_field.getSFRotation()
                cur_yaw = float(cur_rot[3])
            except Exception:
                # fallback: we can use stored start + traveled but better to try again next frame
                cur_x, cur_y, cur_yaw = self.start_pos[0], self.start_pos[1], self.start_yaw

            # remaining distance
            remaining = self.total_dist - self.traveled
            step_dist = self.default_velocity * dt
            move_step = min(step_dist, remaining)

            # update traveled and position
            if remaining > 1e-9:
                # move along move_dir in world coordinates
                dx, dy = (self.move_dir * move_step).tolist()
                new_x = cur_x + dx
                new_y = cur_y + dy
                # set new translation (keep z)
                try:
                    self.trans_field.setSFVec3f([new_x, new_y, float(self.z)])
                except Exception:
                    pass
                self.traveled += move_step
            else:
                # already close enough in translation; just ensure position set later
                pass

            # update rotation: integrate angular_speed (signed)
            ang_step = self.angular_speed * dt
            new_yaw = self.normalize_angle(cur_yaw + ang_step)
            # if we've nearly finished both translation and rotation, finalize
            finish_translation = (self.total_dist - self.traveled) <= 1e-4
            # check remaining angular diff to target
            ang_rem = self.normalize_angle(self.target[2] - new_yaw)
            finish_rotation = abs(ang_rem) <= 1e-3

            try:
                self.rot_field.setSFRotation([0.0, 0.0, 1.0, float(new_yaw)])
            except Exception:
                pass

            if finish_translation and finish_rotation:
                # finalize exact pose and reset physics
                self._finalize_arrival()
            # else keep moving next frame
            return

    def set_points(self, points):
        pts = []
        for p in points:
            if len(p) >= 3:
                pts.append((float(p[0]), float(p[1]), float(p[2])))
            else:
                pts.append((float(p[0]), float(p[1]), random.uniform(-math.pi, math.pi)))
        self.points = pts
        self.idx = 0

# ==== 使用示例（初始化处） ====
obstacle = SmoothRandomObstacle(supervisor, def_name="OTHER1", interval=0, z=0.01, default_velocity=0.3, seed=123)
obstacle.start(immediate_move=True)

# ==== 主循环里每帧调用 ====
# obstacle.update()

# ==== Main logic (randomize balls -> init monitoring -> waypoint queue -> non-blocking main loop) ====
randomize_balls(seed=1235, ensure_no_overlap=True)
monitor_simple_init(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT)

# waypoints example (x, y, angle); angle=None means face first then move
North = math.pi / 2
East = 0.0
South = -math.pi / 2
West = math.pi

waypoints = [
  (-0.85, 0.85, None),
  (0.85, 0.75, None),
  (-0.85, 0.65, None),
  (0.85, 0.55, None),
  (-0.85, 0.45, None),
  (0.85, 0.35, None),
  (-0.85, 0.25, None),
  (0.85, 0.15, None),
  (-0.85, 0.05, None),
  (0.85, -0.05, None),
  (-0.85, -0.15, None),
  (0.85, -0.25, None),
  (-0.85, -0.35, None),
  (0.85, -0.45, None),
  (-0.85, -0.55, None),
  (0.85, -0.65, None),
  (-0.85, -0.75, None),
  (0.85, -0.85, None),
  (-0.70,0, East),
  (0.85,   -0.85,  None),
  (0.75,   0.85,   None),
  (0.65,   -0.85,  None),
  (0.55,   0.85,   None),
  (0.45,   -0.85,  None),
  (0.35,   0.85,   None),
  (0.25,   -0.85,  None),
  (0.15,   0.85,   None),
  (0.05,   -0.85,  None),
  (-0.05,  0.85,   None),
  (-0.15,  -0.85,  None),
  (-0.25,  0.85,   None),
  (-0.35,  -0.85,  None),
  (-0.45,  0.85,   None),
  (-0.55,  -0.85,  None),
  (-0.65,  0.85,   None),
  (-0.75,  -0.85,  None),
  (-0.85,  0.85,   None),
  (-0.70,0, East),
]

motion = MotionController(trans, rot_field, dt)

# Start the first task (if exists)
if len(waypoints) > 0:
    x,y,ang = waypoints.pop(0)
    motion.start(x,y,velocity=None, angle=ang)

# Non-blocking main loop
while supervisor.step(TIME_STEP) != -1:
    # 1) Advance motion (non-blocking)
    motion.update()

    # 2) Call simplified absorption check each frame
    monitor_simple_step(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION)

    # 3) If the current task is done and there is a next waypoint, start it
    if not motion.active and len(waypoints) > 0:
        x,y,ang = waypoints.pop(0)
        motion.start(x,y,velocity=None, angle=ang)

    # 4) Other per-frame logic can be added here (logging / keyboard / debug)
    SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1
    obstacle.update()

# Exit
print("Supervisor controller exiting.")