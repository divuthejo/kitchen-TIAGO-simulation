"""TIAGo autonomous pick-and-place: three jars from the kitchen counter to the table.

Deliberately simple:
  * localisation = GPS (corrected for its 0.2 m mounting offset) + IMU yaw
  * jar positions are the fixed ones from kitchen.wbt
  * all driving happens on a north-south "lane" at x = LANE_X between counter and table
  * two arm poses: REACH (grasp / place, ~1.07 m ahead) and CARRY (driving, ~0.77 m ahead);
    height is set only with the torso lift joint
  * lidar (controllers/common/lidar_guard.py): while driving forward the robot steers around obstacles
    (free heading closest to the goal); every forward wheel command is also checked, so it slows near
    obstacles and, if no way around exists, stops 0.25 m short and waits until the way is clear
  * fingertip tactile pads (controllers/common/tactile.py): grasp until both pads touch, squeeze to
    GRIP_HOLD, tighten automatically if the jar starts to slip (never above GRIP_FORCE_LIMIT), detect a
    dropped jar, and place by FEEL - lower slowly and release when the table takes the jar's weight

Afterwards the three cereal boxes go to the table the same way: each is approached along its own
angle so its 20 cm side lies straight across the wide-open fingers, and is held at the 10 N limit.

Sequence per object: lane -> face jar -> REACH -> creep in -> squeeze (force-limited) -> lift ->
back to lane -> CARRY -> drive to table spot -> face table -> REACH -> lower -> release -> back.
"""
import math
import os
import sys
from controller import Robot

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
from tactile import Tactile  # noqa: E402
from lidar_guard import LidarGuard  # noqa: E402

# ---------------------------------------------------------------- task layout (world frame, m)
JARS = [  # name, x, y   (fixed positions in kitchen.wbt; jar origin = bottom centre)
    ('honey jar', 1.714, 0.704),
    ('jam jar (back)', 1.926, 0.494),   # behind the honey jar, so it goes second
    ('jam jar (red lid)', 1.712, -0.302),
]
SPOTS = [(-0.415, -0.80), (-0.415, -1.75), (-0.415, -2.00)]   # where the jars go on the table
# Cereal boxes: OFF by default. The approach + 10 N grasp works, but a 30 cm, 1 kg box held by two
# 5 cm pads twists out of the grip within seconds of lifting (tested 2026-11-21: 1 of 3 reached the
# table). Set True to try it.
CEREAL_ENABLED = False
CEREAL = [  # name, x, y, yaw   (fixed poses; 0.08 x 0.20 x 0.30 m, 1 kg, standing on the counter)
    ('cereal box (1)', 2.025, 0.334, -2.619),
    ('cereal box (2)', 2.053, 0.038, 2.879),
    ('cereal box (3)', 1.860, -0.607, 2.880),
]


def box_heading(yaw):
    """Approach heading that puts the box's 20 cm side straight across the open fingers."""
    return (yaw + math.pi / 2) % math.pi - math.pi / 2
# Boxes go on the table's east edge, held across their 20 cm side (long side along y when placed).
# Checked against the jars, carafe, water bottle and biscuit box already there.
CEREAL_SPOTS = [(-0.27, -0.65), (-0.27, -0.98), (-0.27, -2.20)]
JAR_GRASP_Z = 0.075      # grasp height above the object's bottom (jar is 0.115 tall)
BOX_GRASP_Z = 0.15       # cereal box: mid-height
LANE_X = 0.65            # x of the driving lane
COUNTER_TOP = 0.889      # z of the jar bottoms on the counter
TABLE_TOP = 0.74
TABLE = (-1.15, -0.15, -2.33, -0.53)   # x_min, x_max, y_min, y_max of the table top
# The scanner (0.2 m above the floor) sees only the table's four legs, not its top, so the avoidance
# would happily drive under it. Its outline is added as known obstacle points.
TABLE_OUTLINE = ([(TABLE[1], TABLE[2] + i * 0.03) for i in range(61)] +
                 [(TABLE[0] + i * 0.03, TABLE[2]) for i in range(34)] +
                 [(TABLE[0] + i * 0.03, TABLE[3]) for i in range(34)])

# ---------------------------------------------------------------- robot geometry (measured in sim)
GPS_OFFSET = 0.202       # GPS sits this far ahead of the robot origin
REACH_X = 1.09           # grasp point (finger centre) ahead of robot origin in the REACH pose
GRASP_Z0 = 0.752         # world z of the grasp point in REACH = GRASP_Z0 + torso
STANDOFF = 0.12          # stop this far short, then creep in

POSES = {   # arm_1..arm_7
    'reach': [1.60, 0.30, 0.00, 0.40, 1.57, 0.01, 0.00],
    'carry': [1.60, 1.02, 0.00, 2.20, 1.57, 1.31, 0.00],
}
TORSO_CARRY = 0.35

# ---------------------------------------------------------------- tactile grip (same as teleop)
GRIP_FORCE_LIMIT = 10.0  # N  safety threshold, never exceeded
GRIP_FORCE_START = 2.0
GRIP_FORCE_RATE = 4.0    # N/s
GRIP_HOLD = 8.0          # N  normal squeeze once both pads touch (6 N let a jar slip out while turning)
SLIP_RATIO = 0.6         # tighten when shear/normal on a pad exceeds this (friction mu = 1)
TOUCHDOWN = 0.3          # placing: released when the carried load drops below 30 % of the held load

# ---------------------------------------------------------------- control gains
MAX_WHEEL = 5.0          # rad/s
WHEEL_R = 0.0985         # m
TURN_TOL = 0.004         # rad
POS_TOL = 0.01           # m

robot = Robot()
TS = int(robot.getBasicTimeStep())
DT = TS / 1000.0

gps = robot.getDevice('gps'); gps.enable(TS)
imu = robot.getDevice('imu'); imu.enable(TS)
wheels = [robot.getDevice('wheel_left_joint'), robot.getDevice('wheel_right_joint')]
for w in wheels:
    w.setPosition(float('inf')); w.setVelocity(0.0)
torso = robot.getDevice('torso_lift_joint')
torso_s = robot.getDevice('torso_lift_joint_sensor'); torso_s.enable(TS)
ARM = ['arm_%d_joint' % i for i in range(1, 8)]
arm = [robot.getDevice(n) for n in ARM]
arm_s = [robot.getDevice(n + '_sensor') for n in ARM]
for s in arm_s:
    s.enable(TS)
fingers = [robot.getDevice('gripper_left_finger_joint'), robot.getDevice('gripper_right_finger_joint')]
finger_s = [robot.getDevice('gripper_left_sensor_finger_joint'), robot.getDevice('gripper_right_sensor_finger_joint')]
for f, s in zip(fingers, finger_s):
    f.enableForceFeedback(TS); s.enable(TS)
FINGER_MAX_FORCE = fingers[0].getMaxForce()
FINGER_OPEN = fingers[0].getMaxPosition()   # full opening (boxes)
JAR_OPEN = 0.06     # per finger: 12 cm opening for the 9 cm jars (wider brushes the cereal box beside them)
   # 0.06 with TiagoGripperWide (12 cm opening)
for h in ('head_1_joint', 'head_2_joint'):
    robot.getDevice(h).setPosition(0.0)
tac = Tactile(robot, TS)
guard = LidarGuard(robot, TS, stop=0.25, slow=0.60)
guard_cfg = {'stop': 0.25, 'slow': 0.60}     # approach() tightens these near the counter / table
drive_state = {'avoiding': False}
grip = {'holding': False, 'cmd': 0.0, 'tightened': 0}


def log(msg):
    print(f'[auto {robot.getTime():6.1f}s] {msg}', flush=True)


def step():
    if robot.step(TS) == -1:
        raise SystemExit
    tac.update()
    if grip['holding']:
        # slip reflex: more squeeze while the pads report the jar sliding, capped at the limit
        if tac.slip > SLIP_RATIO and grip['cmd'] < GRIP_FORCE_LIMIT:
            grip['cmd'] = min(GRIP_FORCE_LIMIT, grip['cmd'] + GRIP_FORCE_RATE * DT)
            grip['tightened'] += 1
            for f in fingers:
                f.setAvailableForce(grip['cmd'])
        if not tac.any_touching:
            log('TACTILE: object lost - both pads read no contact')
            grip['holding'] = False


def wait(sec):
    for _ in range(int(sec / DT)):
        step()


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pose():
    g = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    return g[0] - GPS_OFFSET * math.cos(yaw), g[1] - GPS_OFFSET * math.sin(yaw), yaw


def _wheels(v, w):
    half_track = 0.2022
    l = (v - w * half_track) / WHEEL_R
    r = (v + w * half_track) / WHEEL_R
    k = max(1.0, abs(l) / MAX_WHEEL, abs(r) / MAX_WHEEL)
    wheels[0].setVelocity(l / k); wheels[1].setVelocity(r / k)


def set_wheels(v, w):
    """v: m/s forward, w: rad/s (ccw). Forward motion goes through the lidar stop."""
    if v > 0:
        factor, clear = guard.speed_factor(guard_cfg['stop'], guard_cfg['slow'])
        if factor == 0.0:
            _wheels(0, 0)
            log(f'LIDAR STOP: obstacle {clear:.2f} m ahead - waiting for the path to clear')
            waited = 0.0
            while guard.speed_factor(guard_cfg['stop'] + 0.05, guard_cfg['slow'])[0] == 0.0:
                step(); waited += DT
                if waited % 5.0 < DT:
                    log(f'LIDAR STOP: still blocked ({guard.clearance():.2f} m), waited {waited:.0f} s')
            log(f'LIDAR STOP: path clear after {waited:.1f} s - continuing')
            factor = guard.speed_factor(guard_cfg['stop'], guard_cfg['slow'])[0]
        v *= factor
    _wheels(v, w)


def stop():
    set_wheels(0, 0)
    wait(0.3)


# ---------------------------------------------------------------- base motions
def turn_to(yaw_goal):
    for _ in range(int(20 / DT)):
        err = wrap(yaw_goal - pose()[2])
        if abs(err) < TURN_TOL:
            break
        w = max(-0.5, min(0.5, 2.5 * err))
        if abs(w) < 0.05:
            w = math.copysign(0.05, w)
        set_wheels(0, w); step()
    stop()


def drive_to(x, y, speed=0.3, reverse=False):
    """Turn to face (x, y) (or away from it if reverse) and drive there, steering continuously."""
    px, py, yaw = pose()
    bearing = math.atan2(y - py, x - px)
    if math.hypot(x - px, y - py) < POS_TOL:
        return
    turn_to(wrap(bearing + math.pi) if reverse else bearing)
    for _ in range(int(60 / DT)):
        px, py, yaw = pose()
        dx, dy = x - px, y - py
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading = wrap(yaw + math.pi) if reverse else yaw
        along = dx * math.cos(heading) + dy * math.sin(heading)   # distance left along heading
        v = min(speed, 0.8 * dist + 0.03) * (-1 if reverse else 1)
        w = 2.0 * wrap(bearing - heading) if dist > 0.05 else 0.0
        if not reverse and dist > 0.3:
            # Reactive layer (lidar sees forward only). Look 1.4 m ahead so a detour is chosen while an
            # obstacle is still out of reach of the carried jar (held 0.8 m ahead at chair-back height).
            rel = wrap(bearing - heading)
            look = min(1.4, dist + 0.05)
            ang, room = guard.steer(rel, look, extra=table_points())
            goal_free = guard.free_toward(rel, look, extra=table_points())
            if goal_free < look - 1e-6 and goal_free > dist - 0.6:
                room = 0.0      # the obstacle sits at the destination itself: going round cannot help
            if room < look - 1e-6:
                # no gap wide enough for the base (e.g. a chair in the middle of the lane): stop and wait
                _wheels(0, 0)
                log(f'LIDAR STOP: no way around ({room:.2f} m free) - waiting for the path to clear')
                waited = 0.0
                while guard.free_toward(rel, look, extra=table_points()) < look - 1e-6:
                    step(); waited += DT
                log(f'LIDAR STOP: path clear after {waited:.1f} s - continuing')
                continue
            avoiding = abs(ang - rel) > 0.02
            if avoiding and not drive_state['avoiding']:
                log(f'AVOID: path to goal blocked - steering {math.degrees(ang):+.0f} deg around it')
            if drive_state['avoiding'] and not avoiding:
                log('AVOID: clear again - heading for the goal')
            drive_state['avoiding'] = avoiding
            if dist < POS_TOL or (along < 0 and not avoiding):
                break
            w = max(-0.6, min(0.6, 2.0 * ang))
            if guard.clearance() < guard_cfg['stop'] or abs(ang) > 0.6:
                v = 0.0                                   # turn on the spot toward the free heading
            _wheels(v, w); step()
            continue
        if dist < POS_TOL or along < 0:
            break
        if not reverse and guard.clearance() > dist + 0.03:
            _wheels(v, w); step()          # final stretch: the obstacle (if any) is beyond the stop point
        else:
            set_wheels(v, w); step()
    drive_state['avoiding'] = False
    stop()


def table_points():
    """The table outline in the robot frame (for the avoidance, which cannot see the table top)."""
    px, py, yaw = pose()
    c, s_ = math.cos(yaw), math.sin(yaw)
    return [((x - px) * c + (y - py) * s_, -(x - px) * s_ + (y - py) * c) for x, y in TABLE_OUTLINE]


def go_to(x, y, reverse=False):
    """drive_to, repeated: turning in place lets the wheels slip a few cm, so re-check and correct."""
    for _ in range(4):
        px, py, _ = pose()
        if math.hypot(x - px, y - py) < 0.02:
            return
        drive_to(x, y, reverse=reverse)


def go_lane(y):
    """Move along the lane to (LANE_X, y). Never face north while y > 0.2 (sink/counter behind)."""
    north = (y - pose()[1]) > 0
    go_to(LANE_X, y, reverse=north)


def face(target_xy):
    turn_to(math.atan2(target_xy[1] - pose()[1], target_xy[0] - pose()[0]))


def approach(target_xy, dist_goal, speed=0.04):
    """Move forward OR backward, staying pointed at target, until the robot origin is dist_goal from it."""
    tx, ty = target_xy
    face(target_xy)
    # final approach to a known target (counter / table): only stop for something really close
    guard_cfg.update(stop=0.12, slow=0.12)
    for _ in range(int(40 / DT)):
        px, py, yaw = pose()
        err = math.hypot(tx - px, ty - py) - dist_goal
        if abs(err) < 0.004:
            break
        v = math.copysign(min(speed, abs(err) + 0.005), err)
        w = 2.0 * wrap(math.atan2(ty - py, tx - px) - yaw)
        set_wheels(v, w); step()
    guard_cfg.update(stop=0.25, slow=0.60)
    stop()
    face(target_xy)


def back_off(dist, speed=0.1):
    x0, y0, _ = pose()
    for _ in range(int(30 / DT)):
        px, py, _ = pose()
        if math.hypot(px - x0, py - y0) >= dist:
            break
        set_wheels(-speed, 0); step()
    stop()


# ---------------------------------------------------------------- arm, lift, gripper
def move_torso(t, timeout=8.0):
    t = max(0.0, min(0.35, t))
    torso.setPosition(t)
    for _ in range(int(timeout / DT)):
        step()
        if abs(torso_s.getValue() - t) < 0.003:
            break


def move_joints(goal, rate=0.5):
    """Interpolate all arm joints together. Letting each motor run at its own speed makes the
    gripper dip ~24 cm half-way from CARRY to REACH (measured) - enough to hit the counter/table."""
    start = [s.getValue() for s in arm_s]
    secs = max(1.0, max(abs(g - s) for g, s in zip(goal, start)) / rate)
    n = int(secs / DT)
    for i in range(1, n + 1):
        a = i / n
        for m, s0, g in zip(arm, start, goal):
            m.setPosition(s0 + (g - s0) * a)
        step()
    wait(0.5)


def move_arm(name=None, torso_pos=None):
    """Going up: lift first, then arm. Going down: arm first, then lift (never lower while sweeping)."""
    up = torso_pos is not None and torso_pos > torso_s.getValue()
    if up:
        move_torso(torso_pos)
    if name is not None:
        move_joints(POSES[name])
    if torso_pos is not None and not up:
        move_torso(torso_pos)
    wait(0.3)


def open_gripper(width=JAR_OPEN):
    grip['holding'] = False
    for f in fingers:
        f.setAvailableForce(FINGER_MAX_FORCE); f.setPosition(min(width, FINGER_OPEN))
    wait(1.0)


def grasp(hold=GRIP_HOLD):
    """Close gently until BOTH pads touch, then squeeze to `hold` (never above the limit)."""
    cmd = GRIP_FORCE_START
    for _ in range(int(8 / DT)):
        if tac.both_touching and cmd >= hold:
            break
        if tac.both_touching or cmd < hold:
            cmd = min(hold, cmd + GRIP_FORCE_RATE * DT)
        for f in fingers:
            f.setAvailableForce(min(cmd, GRIP_FORCE_LIMIT)); f.setPosition(0.0)
        step()
    wait(0.5)
    gap = sum(s.getValue() for s in finger_s)
    holding = tac.both_touching and gap > 0.02
    grip.update(holding=holding, cmd=cmd, tightened=0)
    log(f'grasp: {tac.summary()}, fingers L {finger_s[0].getValue() * 100:.1f} R '
        f'{finger_s[1].getValue() * 100:.1f} cm -> {"HOLDING" if holding else "MISSED"}')
    return holding


def lower_until_touchdown(t_expected):
    """Lower the lift slowly and stop the moment the table takes the jar's weight."""
    held = tac.load
    move_torso(t_expected + 0.04)
    torso.setVelocity(0.01)
    torso.setPosition(max(0.0, t_expected - 0.03))
    felt = False
    for _ in range(int(6 / DT)):
        step()
        if tac.load < TOUCHDOWN * held:
            felt = True
            break
        if abs(torso_s.getValue() - max(0.0, t_expected - 0.03)) < 0.002:
            break
    torso.setPosition(torso_s.getValue())
    torso.setVelocity(torso.getMaxVelocity())
    log(f'place: {"touchdown FELT" if felt else "no touchdown felt"} at lift {torso_s.getValue():.3f} '
        f'(model said {t_expected:.3f}); load {held:.1f} -> {tac.load:.1f} N')
    wait(0.2)
    return felt


def torso_for(grasp_z):
    return grasp_z - GRASP_Z0


# ---------------------------------------------------------------- mission
def line_up(tx, ty, dist, heading=None):
    """Stand dist away from the target, pointing at it: on the lane at the target's y, or - if heading
    is given - on the line through the target along that heading. Verified and retried if off."""
    h = 0.0 if heading is None else heading
    for attempt in range(3):
        if heading is None:
            go_lane(ty)
        else:
            sx, sy = tx - dist * math.cos(h), ty - dist * math.sin(h)
            go_to(sx, sy, reverse=(sy - pose()[1]) > 0)
        approach((tx, ty), dist, speed=0.15)
        px, py, yaw = pose()
        off_line = abs(py - ty) if heading is None else abs(-(px - tx) * math.sin(h) + (py - ty) * math.cos(h))
        off_aim = abs(wrap(math.atan2(ty - py, tx - px) - yaw))
        if off_line < 0.03 and off_aim < 0.05:
            return
        log(f'line-up off by {off_line * 100:.1f} cm / {math.degrees(off_aim):.1f} deg at '
            f'({px:.3f}, {py:.3f}) - redoing (attempt {attempt + 1})')
        back_off(0.15)


def pick(name, jx, jy, grasp_z=JAR_GRASP_Z, heading=None, hold=None):
    log(f'--- picking {name} at ({jx:.3f}, {jy:.3f})')
    # stand off so that, once in REACH, the fingers are STANDOFF short of the object
    line_up(jx, jy, REACH_X + STANDOFF, heading)
    open_gripper(JAR_OPEN if heading is None else FINGER_OPEN)
    t_grasp = torso_for(COUNTER_TOP + grasp_z)
    move_arm('reach', t_grasp)
    approach((jx, jy), REACH_X)
    ok = grasp(hold or GRIP_HOLD)
    move_arm(None, t_grasp + 0.10)          # lift it off the counter
    if ok:
        log(f'lifted: {tac.summary()}')
        ok = grip['holding']
    back_off(0.25)
    move_arm('carry', TORSO_CARRY)
    return ok


def place(sx, sy, grasp_z=JAR_GRASP_Z):
    log(f'--- placing at ({sx:.3f}, {sy:.3f})')
    line_up(sx, sy, REACH_X + STANDOFF)
    t_place = torso_for(TABLE_TOP + grasp_z)   # where the object should touch down
    move_arm('reach', max(t_place + 0.10, 0.25))   # extend above the table first
    approach((sx, sy), REACH_X)
    if grip['tightened']:
        log(f'slip reflex tightened the grip during transport (now {grip["cmd"]:.1f} N)')
    felt = lower_until_touchdown(t_place)
    open_gripper()
    move_arm(None, t_place + 0.10)
    back_off(0.25)
    move_arm('carry', TORSO_CARRY)
    return felt


def main():
    step()
    log('start: pose x=%.3f y=%.3f yaw=%.3f' % pose())
    open_gripper()
    move_arm('carry', TORSO_CARRY)
    done = {'jars': 0, 'cereal boxes': 0}
    tasks = [('jars', name, x, y, sp, JAR_GRASP_Z, None, None) for (name, x, y), sp in zip(JARS, SPOTS)]
    if CEREAL_ENABLED:
        tasks += [('cereal boxes', name, x, y, sp, BOX_GRASP_Z, box_heading(yaw), GRIP_FORCE_LIMIT)
                  for (name, x, y, yaw), sp in zip(CEREAL, CEREAL_SPOTS)]
    for kind, name, ox, oy, (sx, sy), gz, heading, hold in tasks:
        if pick(name, ox, oy, gz, heading, hold):
            if place(sx, sy, gz):          # counts only if the table was felt taking the weight
                done[kind] += 1
        else:
            open_gripper()
            log(f'skipping {name}: grasp failed')
    go_lane(0.2)
    msg = f'MISSION DONE: {done["jars"]}/{len(JARS)} jars placed'
    if CEREAL_ENABLED:
        msg += f', {done["cereal boxes"]}/{len(CEREAL)} cereal boxes placed'
    log(msg)
    while robot.step(TS) != -1:
        pass


main()
