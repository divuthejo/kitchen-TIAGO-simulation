"""Fingertip tactile sensing for the TIAGo gripper (shared by tiago_auto and tiago_ps4).

Each finger pad of protos/TiagoGripperWide.proto is a force-3d TouchSensor that reports signed
newtons in the pad frame (measured in sim):
    x = normal force (the squeeze)                 holding a jar at 10 N -> L.x = R.x = 10
    y = vertical shear (the pads mirror each other) holding a 0.5 kg jar -> R.y - L.y = +4.9 N
    z = shear along the finger
From that we get:
    grip     - squeeze per pad (N)
    load     - weight the fingers are carrying (N): ~mg while holding in the air, ~0 when the object
               rests on a surface, negative when it is pushed into one  -> touchdown detection
    slip     - friction utilisation |shear| / normal per pad (1.0 = about to slip, world friction mu = 1)
"""
import math

CONTACT_N = 0.5          # a pad touching something reads at least this much normal force


class Tactile:
    def __init__(self, robot, timestep):
        self.pads = [robot.getDevice('gripper_left_tactile'), robot.getDevice('gripper_right_tactile')]
        self.ok = all(self.pads)
        if self.ok:
            for p in self.pads:
                p.enable(timestep)
        else:
            print('[tactile] finger TouchSensors not found - is the robot using TiagoGripperWide?')
        self.L = self.R = (0.0, 0.0, 0.0)

    def update(self):
        if self.ok:
            # NB: getValues() of a TouchSensor returns a raw C pointer in the R2025a Python API;
            # iterating it without slicing reads past the end and segfaults.
            self.L = tuple(self.pads[0].getValues()[:3])
            self.R = tuple(self.pads[1].getValues()[:3])
        return self

    @property
    def grip(self):
        return self.L[0], self.R[0]

    @property
    def load(self):
        return self.R[1] - self.L[1]

    @property
    def slip(self):
        def ratio(p):
            return math.hypot(p[1], p[2]) / max(p[0], 0.1)
        return max(ratio(self.L), ratio(self.R))

    @property
    def both_touching(self):
        return min(self.grip) > CONTACT_N

    @property
    def any_touching(self):
        return max(self.grip) > CONTACT_N

    def summary(self):
        slip = f'slip {self.slip:4.2f}' if self.both_touching else 'slip  -  '
        return f'pads L {self.L[0]:4.1f} N / R {self.R[0]:4.1f} N  load {self.load:+5.1f} N  {slip}'
