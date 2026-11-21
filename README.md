# TIAGo kitchen pick-and-place (Webots R2025a)

The Webots `kitchen.wbt` sample with a PAL TIAGo that moves the three jars from the counter to the table,
plus a PS4 teleop mode. 
## Run

```bash
webots worlds/kitchen_tiago.wbt          # autonomous: 3 jars counter -> table (~6 min sim time)
webots worlds/kitchen_tiago_teleop.wbt   # drive it yourself with a PS4 pad (or the keyboard)
```

## What it does

- **Autonomous** (`controllers/tiago_auto`): GPS + IMU localisation, a driving lane between counter and
  table, two arm poses (reach / carry) with height from the torso lift. Tested 3/3 jars, within ~2 mm.
- **Tactile sensing** (`controllers/common/tactile.py`): the finger pads are 3-axis force sensors.
  Grip force capped at 10 N, slip reflex, drop detection, and objects are placed by feeling the table.
- **Lidar** (`controllers/common/lidar_guard.py`): slows near obstacles, stops 0.25 m short and waits
  until the path is clear; steers around obstacles in the middle of the path (the table outline is added
  as a known obstacle because the scanner only sees its legs).
- **PS4 teleop** (`controllers/tiago_ps4`): see the docstring at the top of the file for the controls.
  

## Robot changes (`protos/`)

- `TiagoGripperWide.proto`: wider finger travel (24 cm max) and force-3d TouchSensor finger pads.
- `Tiago.proto` + `TiagoBase.proto`: local copies whose caster wheels use a low-friction contact
  material. With the stock casters the base slid 20-35 cm on every turn in place.
- `HoneyJar` / `JamJar` / `CerealBox`: copies with a `recognitionColors` field for the camera.
