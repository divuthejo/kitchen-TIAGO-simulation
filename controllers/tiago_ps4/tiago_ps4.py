"""TIAGo PS4 teleop for the kitchen world, with a tactile grip-force safety limit.

Controls (PS4, Linux hid-sony layout; run with DEBUG_JOY=True to check your indices):
  Left stick Y ........ drive forward / back
  Right stick X ....... turn
  D-pad up / down ..... torso lift up / down (the "lift joint")
  R2 (hold) ........... close gripper; keep holding to squeeze harder
                        -> squeeze force ramps up but STOPS at GRIP_FORCE_LIMIT
  L2 .................. open gripper (releases, resets squeeze force)
  Triangle ............ arm preset "reach" (gripper level, 1.09 m ahead - grasp / place)
  Square .............. arm preset "carry" (gripper level, 0.77 m ahead - drive with a jar)
  Cross ............... arm preset "tuck"  (arm folded)
  Options ............. toggle ARM-JOG mode (left stick Y jogs the selected joint)
  L1 / R1 ............. (arm-jog mode) select previous / next arm joint
  Share ............... print all joint positions (copy them into PRESETS)

Keyboard fallback (click the 3D view first): arrows drive, W/S lift, C close, O open,
R reach, T carry, U tuck.

Tactile layer (controllers/common/tactile.py): each fingertip pad is a 3-axis force sensor.
  * safety limit - the squeeze command is applied as the motors' available force and stops rising
    when it, or either pad's measured normal force, reaches GRIP_FORCE_LIMIT, however long R2 is held
  * slip reflex  - while gripping, if a pad reports the object sliding, the grip tightens on its own
    (still capped by the limit)
  * surface feel - while holding, if the object's weight comes off the fingers (it is resting on a
    surface) you get a message + rumble: safe to release
Lidar stop (controllers/common/lidar_guard.py): driving forward slows near obstacles and stops 0.25 m
short (turning and reversing still work); the pad rumbles when it blocks you.
Arm presets move all joints together (moving them independently dips the gripper ~24 cm mid-way).
"""
import os
import sys
from controller import Robot, Keyboard

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
from tactile import Tactile  # noqa: E402
from lidar_guard import LidarGuard  # noqa: E402

# ---------------------------------------------------------------- tuning
GRIP_FORCE_LIMIT = 10.0   # N  safety threshold (finger motor max is 16 N)
GRIP_FORCE_START = 2.0    # N  force when you first press R2 (fingers close gently)
GRIP_FORCE_RATE = 4.0     # N/s squeeze increase while R2 is held
SLIP_RATIO = 0.6          # pad shear/normal above this = object starting to slip
ARM_RATE = 0.5            # rad/s for the fastest joint during a preset move
MAX_SPEED = 6.0           # rad/s wheel speed at full stick
LIFT_SPEED = 0.05         # m/s torso
JOG_SPEED = 0.6           # rad/s arm jog
DEADZONE = 0.12
DEBUG_JOY = False         # True: print raw axes/povs/buttons on change (+ joy_debug.log) to check a mapping

# Webots Joystick indices for this PS4 pad on Linux (measured 2026-11-21 with DEBUG_JOY): 6 axes
# (LX, LY, L2, RX, RY, R2; stick up = -1), D-pad on POV 0 (up 1, down 16), buttons Cross 0, Circle 1,
# Triangle 2, Square 3, L1 4, R1 5, L2 6, R2 7, Share 8, Options 9.
AX_LX, AX_LY, AX_L2, AX_RX, AX_RY, AX_R2 = range(6)
BTN_CROSS, BTN_O, BTN_TRI, BTN_SQ, BTN_L1, BTN_R1, BTN_L2, BTN_R2, BTN_SHARE, BTN_OPT = range(10)

ARM = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint',
       'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
PRESETS = {
    # torso, arm_1..arm_7  (single arm configuration; height comes from the lift joint)
    # same poses as tiago_auto; lift 0.2 puts the fingers at jar height on the counter
    'reach': [0.35, 1.60, 0.30, 0.00, 0.40, 1.57, 0.01, 0.00],
    'carry': [0.35, 1.60, 1.02, 0.00, 2.20, 1.57, 1.31, 0.00],
    'tuck':  [0.10, 0.07, 1.02, -3.16, 1.27, 1.32, 0.00, 1.41],
}

robot = Robot()

# mirror every console message into tiago_ps4.log (next to this file) so a session can be reviewed later
_log_file = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tiago_ps4.log'), 'w')
_print = print


def print(*args, **kwargs):  # noqa: A001 - deliberately wraps the builtin
    _print(*args, **kwargs)
    _log_file.write(f'[{robot.getTime():7.2f}s] ' + ' '.join(str(a) for a in args) + '\n')
    _log_file.flush()

dt_ms = int(robot.getBasicTimeStep())
dt = dt_ms / 1000.0


def dev(name):
    d = robot.getDevice(name)
    if d is None:
        print(f'[tiago_ps4] missing device: {name}')
    return d


wheels = [dev('wheel_left_joint'), dev('wheel_right_joint')]
for w in wheels:
    w.setPosition(float('inf'))
    w.setVelocity(0.0)

torso = dev('torso_lift_joint')
torso_sensor = dev('torso_lift_joint_sensor')
arm = [dev(n) for n in ARM]
arm_sensors = [dev(n + '_sensor') for n in ARM]
fingers = [dev('gripper_left_finger_joint'), dev('gripper_right_finger_joint')]
finger_sensors = [dev('gripper_left_sensor_finger_joint'), dev('gripper_right_sensor_finger_joint')]
FINGER_MAX_FORCE = fingers[0].getMaxForce()
FINGER_OPEN = fingers[0].getMaxPosition()   # 0.06 with TiagoGripperWide (12 cm opening)
for s in [torso_sensor] + arm_sensors + finger_sensors:
    s.enable(dt_ms)
for f in fingers:
    f.enableForceFeedback(dt_ms)
tac = Tactile(robot, dt_ms)

# perception / navigation sensors (enabled so they are ready for the autonomous layers)
guard = LidarGuard(robot, dt_ms, stop=0.25, slow=0.60)
blocked_announced = False
for name in ('gps', 'compass', 'imu'):
    d = robot.getDevice(name)
    if d:
        d.enable(dt_ms)
camera = robot.getDevice('camera')
if camera:
    camera.enable(dt_ms * 4)
    camera.recognitionEnable(dt_ms * 4)
    camera.enableRecognitionSegmentation()

joy = robot.getJoystick()
joy.enable(dt_ms)
kb = robot.getKeyboard()
kb.enable(dt_ms)

# ---------------------------------------------------------------- state
torso_target = 0.0
arm_targets = [0.0] * len(ARM)
arm_cmd = None            # what the motors are currently told (moves toward arm_targets)
peak_load = 0.0
surface_announced = False
slip_announced = False
grip_closing = False
grip_force_cmd = 0.0
limit_announced = False
jog_mode = False
jog_idx = 0
prev_buttons = set()
print_timer = 0.0
joy_announced = False
last_raw = None
joy_log = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joy_debug.log'), 'w') if DEBUG_JOY else None


def apply_preset(name):
    global torso_target, arm_targets
    p = PRESETS[name]
    torso_target = p[0]
    arm_targets = list(p[1:])
    print(f'[tiago_ps4] preset "{name}"')


def open_gripper():
    global grip_closing, grip_force_cmd, limit_announced, peak_load, surface_announced, slip_announced
    grip_closing, grip_force_cmd, limit_announced = False, 0.0, False
    peak_load, surface_announced, slip_announced = 0.0, False, False
    for f in fingers:
        f.setAvailableForce(FINGER_MAX_FORCE)
        f.setPosition(FINGER_OPEN)


def rumble(seconds):
    """Short rumble where the pad / driver supports force feedback (silently ignored otherwise)."""
    if joy.isConnected():
        try:
            joy.setConstantForceDuration(seconds)
            joy.setConstantForce(10000)
        except Exception:
            pass


def axis(i):
    if i >= joy.getNumberOfAxes():
        return 0.0
    v = joy.getAxisValue(i) / 32768.0
    return 0.0 if abs(v) < DEADZONE else max(-1.0, min(1.0, v))


def trigger(btn, buttons):
    """L2/R2 as on/off. The pad also reports them as axes 2/5, but on Linux those read 0 until the
    first press and -1 afterwards, so the digital buttons (6/7) are the reliable signal."""
    return 1.0 if btn in buttons else 0.0


def dpad_vertical():
    """+1 up, -1 down. Measured on this pad: POV 0 reads 1 for up and 16 for down."""
    for i in range(joy.getNumberOfPovs()):
        v = joy.getPovValue(i)
        if v & 1:
            return 1.0
        if v & 16:
            return -1.0
    return 0.0


# wait one step so sensors have values, then start from a safe pose
robot.step(dt_ms)
arm_cmd = [s.getValue() for s in arm_sensors]
apply_preset('tuck')
open_gripper()

# ---------------------------------------------------------------- scripted input (testing only)
# controllerArgs "--script=<file.json>" replaces the pad with timed inputs, so the teleop can be tested
# headless. Each entry: {"t": start_s, "dur": seconds, "buttons": [ids held], "lx"/"ly"/"rx"/"ry": -1..1,
# "dpad": 1 / -1}. Everything after the input stage runs exactly as with the real pad.
SCRIPT = None
for arg in sys.argv[1:]:
    if arg.startswith('--script='):
        import json
        with open(arg.split('=', 1)[1]) as fh:
            SCRIPT = json.load(fh)
        print(f'[tiago_ps4] SCRIPTED INPUT from {arg.split("=", 1)[1]} ({len(SCRIPT)} steps)')


def scripted(now):
    held, val = set(), {}
    for e in SCRIPT:
        if e['t'] <= now < e['t'] + e.get('dur', 0.2):
            held |= set(e.get('buttons', []))
            for k in ('lx', 'ly', 'rx', 'ry', 'dpad'):
                if k in e:
                    val[k] = e[k]
    return held, val


while robot.step(dt_ms) != -1:
    # ------------------------------------------------ read inputs
    buttons = set()
    if SCRIPT is not None:
        buttons, val = scripted(robot.getTime())
        fwd, turn, jog = -val.get('ly', 0.0), -val.get('rx', 0.0), -val.get('ly', 0.0)
        dpad_y = val.get('dpad', 0.0)
        r2, l2 = trigger(BTN_R2, buttons), trigger(BTN_L2, buttons)
    elif joy.isConnected():
        if not joy_announced:
            print(f'[tiago_ps4] joystick: {joy.model} '
                  f'({joy.getNumberOfAxes()} axes, {joy.getNumberOfPovs()} povs)')
            joy_announced = True
        b = joy.getPressedButton()
        while b != -1:
            buttons.add(b)
            b = joy.getPressedButton()
        fwd = -axis(AX_LY)
        turn = -axis(AX_RX)
        dpad_y = dpad_vertical()
        jog = -axis(AX_LY)
        r2 = trigger(BTN_R2, buttons)
        l2 = trigger(BTN_L2, buttons)
        if DEBUG_JOY:
            raw = ([round(joy.getAxisValue(i) / 32768, 1) for i in range(joy.getNumberOfAxes())],
                   [joy.getPovValue(i) for i in range(joy.getNumberOfPovs())], sorted(buttons))
            if raw != last_raw:
                line = f'[joy] t={robot.getTime():7.2f} axes {raw[0]}  povs {raw[1]}  buttons {raw[2]}'
                print(line)
                joy_log.write(line + '\n'); joy_log.flush()
                last_raw = raw
    else:
        fwd = turn = dpad_y = jog = r2 = l2 = 0.0

    keys = set()
    k = kb.getKey()
    while k != -1:
        keys.add(k)
        k = kb.getKey()
    if keys:
        fwd += (Keyboard.UP in keys) - (Keyboard.DOWN in keys)
        turn += (Keyboard.LEFT in keys) - (Keyboard.RIGHT in keys)
        dpad_y += (ord('W') in keys) - (ord('S') in keys)
        r2 = max(r2, float(ord('C') in keys))
        l2 = max(l2, float(ord('O') in keys))
        if ord('R') in keys:
            buttons.add(BTN_TRI)
        if ord('T') in keys:
            buttons.add(BTN_SQ)
        if ord('U') in keys:
            buttons.add(BTN_CROSS)

    pressed = buttons - prev_buttons          # rising edges only
    prev_buttons = buttons

    # ------------------------------------------------ mode / presets
    if BTN_OPT in pressed:
        jog_mode = not jog_mode
        print(f'[tiago_ps4] arm-jog mode {"ON  (joint " + ARM[jog_idx] + ")" if jog_mode else "OFF"}')
    if jog_mode and BTN_R1 in pressed:
        jog_idx = (jog_idx + 1) % len(ARM)
        print(f'[tiago_ps4] jog joint -> {ARM[jog_idx]}')
    if jog_mode and BTN_L1 in pressed:
        jog_idx = (jog_idx - 1) % len(ARM)
        print(f'[tiago_ps4] jog joint -> {ARM[jog_idx]}')
    if BTN_TRI in pressed:
        apply_preset('reach')
    if BTN_SQ in pressed:
        apply_preset('carry')
    if BTN_CROSS in pressed:
        apply_preset('tuck')
    if BTN_SHARE in pressed:
        vals = [round(torso_sensor.getValue(), 3)] + [round(s.getValue(), 3) for s in arm_sensors]
        print(f'[tiago_ps4] current pose [torso, arm_1..7]: {vals}')

    # ------------------------------------------------ base
    if jog_mode:
        fwd_cmd = 0.0
    else:
        fwd_cmd = fwd
    if fwd_cmd > 0:
        factor, clear = guard.speed_factor()
        if factor == 0.0 and not blocked_announced:
            print(f'[tiago_ps4] LIDAR STOP: obstacle {clear:.2f} m ahead - forward blocked (turn or reverse)')
            if True:
                rumble(0.20)
        blocked_announced = factor == 0.0
        fwd_cmd *= factor
    left = (fwd_cmd - 0.5 * turn) * MAX_SPEED
    right = (fwd_cmd + 0.5 * turn) * MAX_SPEED
    wheels[0].setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left)))
    wheels[1].setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right)))

    # ------------------------------------------------ lift + arm
    torso_target = min(0.35, max(0.0, torso_target + dpad_y * LIFT_SPEED * dt))
    torso.setPosition(torso_target)
    if jog_mode and jog:
        m = arm[jog_idx]
        arm_targets[jog_idx] = min(m.getMaxPosition(), max(m.getMinPosition(),
                                   arm_targets[jog_idx] + jog * JOG_SPEED * dt))
    # move all joints together so they arrive at the same time (no mid-way dip)
    deltas = [t - c for t, c in zip(arm_targets, arm_cmd)]
    biggest = max(abs(d) for d in deltas)
    if biggest > 1e-6:
        k = min(1.0, ARM_RATE * dt / biggest)
        arm_cmd = [c + d * k for c, d in zip(arm_cmd, deltas)]
    for m, t in zip(arm, arm_cmd):
        m.setPosition(t)

    # ------------------------------------------------ gripper + tactile layer
    tac.update()
    pads = tac.grip if tac.ok else tuple(abs(f.getForceFeedback()) for f in fingers)
    finger_pos = [s.getValue() for s in finger_sensors]
    at_limit = grip_force_cmd >= GRIP_FORCE_LIMIT or max(pads) >= GRIP_FORCE_LIMIT
    if l2 > 0.5:
        open_gripper()
    elif r2 > 0.1:
        if not grip_closing:
            grip_closing = True
            grip_force_cmd = GRIP_FORCE_START
        # holding R2 asks for more squeeze, but never beyond the safety limit
        if not at_limit:
            grip_force_cmd = min(GRIP_FORCE_LIMIT, grip_force_cmd + r2 * GRIP_FORCE_RATE * dt)
        elif not limit_announced:
            print(f'[tiago_ps4] TACTILE LIMIT {GRIP_FORCE_LIMIT:.1f} N reached - grip will not tighten '
                  f'further (pads read {pads[0]:.1f} / {pads[1]:.1f} N)')
            limit_announced = True
            if True:
                rumble(0.30)   # short rumble where the pad supports it

    if grip_closing:
        if tac.ok and tac.both_touching:
            # slip reflex: tighten on its own while the object slides, still capped by the limit
            if tac.slip > SLIP_RATIO and not at_limit:
                grip_force_cmd = min(GRIP_FORCE_LIMIT, grip_force_cmd + GRIP_FORCE_RATE * dt)
                if not slip_announced:
                    print(f'[tiago_ps4] TACTILE: slip detected (ratio {tac.slip:.2f}) - tightening')
                    slip_announced = True
            # surface feel: the weight came off the fingers -> it is resting on something
            peak_load = max(peak_load, tac.load)
            if peak_load > 2.0 and tac.load < 0.3 * peak_load and not surface_announced:
                print(f'[tiago_ps4] TACTILE: object is resting on a surface (load {peak_load:.1f} -> '
                      f'{tac.load:.1f} N) - safe to release with L2 / O')
                surface_announced = True
                if joy.isConnected():
                    rumble(0.15)
            elif tac.load > 0.7 * peak_load:
                surface_announced = False
        # target "fully closed"; the available force is what the fingertips actually squeeze with
        for f in fingers:
            f.setAvailableForce(grip_force_cmd)
            f.setPosition(0.0)
        # independent watchdog: if a pad ever reads above the limit, back the command off
        if max(pads) > GRIP_FORCE_LIMIT * 1.05:
            grip_force_cmd = max(0.0, grip_force_cmd - GRIP_FORCE_RATE * dt * 4)

    # ------------------------------------------------ status line
    print_timer += dt
    if print_timer >= 1.0 and (grip_closing or jog_mode):
        print_timer = 0.0
        status = (f'grip cmd {grip_force_cmd:4.1f}/{GRIP_FORCE_LIMIT:.0f} N  {tac.summary()}  '
                  f'gap {sum(finger_pos) * 100:4.1f} cm') if grip_closing else ''
        if jog_mode:
            status += f'  jog {ARM[jog_idx]}={arm_sensors[jog_idx].getValue():.2f}'
        print('[tiago_ps4] ' + status.strip())
