"""Lidar stop: a simple reactive safety layer for the TIAGo base (shared by tiago_auto and tiago_ps4).

The Hokuyo on the base scans a horizontal plane ~0.2 m above the floor (sees chair legs, table legs,
cabinets). At the edges of its 240 deg view it also sees the robot's own body, so points inside the
base outline are ignored. We look at the corridor the robot is about to drive through - straight ahead, as wide as
the base plus a margin - and report the free distance in front of the bumper.
  clearance <= stop  -> forward motion must stop (turning in place is still allowed)
  clearance <  slow  -> forward speed is scaled down linearly
The scanner faces forward (240 deg field of view), so it cannot protect reversing.
"""
import math

LIDAR_NAME = 'Hokuyo URG-04LX-UG01'
MOUNT_X = 0.202        # scanner sits this far ahead of the robot origin (same slot as the GPS)
FRONT = 0.27           # base radius: distance from origin to the front bumper


class LidarGuard:
    def __init__(self, robot, timestep, stop=0.25, slow=0.60):
        self.lidar = robot.getDevice(LIDAR_NAME)
        self.ok = self.lidar is not None
        self.stop, self.slow = stop, slow
        if self.ok:
            self.lidar.enable(timestep)
            self.n = self.lidar.getHorizontalResolution()
            fov = self.lidar.getFov()
            # Webots range images run from +fov/2 (left) to -fov/2 (right)
            self.angles = [fov / 2 - i * fov / (self.n - 1) for i in range(self.n)]
            self.cos = [math.cos(a) for a in self.angles]
            self.sin = [math.sin(a) for a in self.angles]
        else:
            print(f'[lidar] device "{LIDAR_NAME}" not found - lidar stop disabled')

    def clearance(self):
        """Free distance (m) between the front bumper and the nearest obstacle in the corridor."""
        if not self.ok:
            return float('inf')
        ranges = self.lidar.getRangeImage()
        best = float('inf')
        for r, c, s in zip(ranges, self.cos, self.sin):
            if not math.isfinite(r) or r <= 0.0:
                continue
            x = MOUNT_X + r * c          # point in the robot frame
            y = r * s
            d = self._distance(x, y)
            if d is not None:
                best = min(best, d)
        return best

    @staticmethod
    def _distance(x, y):
        """How far the round base can drive straight ahead before touching point (x, y), or None if
        driving straight never reaches it. Points inside the base outline are the robot itself."""
        if x <= 0 or math.hypot(x, y) <= FRONT + 0.01:
            return None
        if abs(y) < FRONT:                       # inside the path the round base sweeps going straight
            return x - math.sqrt(FRONT ** 2 - y ** 2)
        return None                              # beside the path (e.g. the counter 0.276 m to the side
                                                 # at the honey jar) - no sideways margin, the stop
                                                 # distance ahead is the safety buffer

    def offenders(self, k=6):
        """Debug: the closest corridor points as (range, angle_deg, x, y)."""
        pts = []
        for r, a, c, s in zip(self.lidar.getRangeImage(), self.angles, self.cos, self.sin):
            if math.isfinite(r) and r > 0:
                x, y = MOUNT_X + r * c, r * s
                if self._distance(x, y) is not None:
                    pts.append((round(r, 3), round(math.degrees(a), 1), round(x, 3), round(y, 3)))
        return sorted(pts, key=lambda p: p[2])[:k]

    def _points(self, extra=()):
        pts = []
        if self.ok:
            for r, c, s in zip(self.lidar.getRangeImage(), self.cos, self.sin):
                if math.isfinite(r) and r > 0:
                    x, y = MOUNT_X + r * c, r * s
                    if math.hypot(x, y) > FRONT + 0.01:     # skip hits on the robot itself
                        pts.append((x, y))
        pts.extend(extra)
        return pts

    @staticmethod
    def _free(a, pts, look):
        """How far the round base can drive along heading a (rad, robot frame), capped at look."""
        ca, sa = math.cos(a), math.sin(a)
        best = look
        for x, y in pts:
            xr, yr = x * ca + y * sa, -x * sa + y * ca          # point in the candidate's frame
            if xr > 0 and abs(yr) < FRONT:
                best = min(best, xr - math.sqrt(FRONT ** 2 - yr ** 2))
        return best

    def free_toward(self, angle, look=1.0, extra=()):
        return self._free(angle, self._points(extra), look)

    def steer(self, goal_angle, look=1.0, step_deg=5, max_deg=70, extra=()):
        """Reactive avoidance: pick a heading (rad, relative to the robot) to drive along.
        Straight at the goal if the base's swept path that way is free for `look` metres, otherwise the
        free heading closest to the goal; if no heading is free, the one with the most room.
        `extra` = additional obstacle points (robot frame) the scanner cannot see, e.g. a table top.
        Returns (angle, free_distance_along_it)."""
        if not self.ok:
            return goal_angle, float('inf')
        pts = self._points(extra)
        g = max(-math.radians(max_deg), min(math.radians(max_deg), goal_angle))
        fg = self._free(g, pts, look)
        if fg >= look - 1e-6:
            return goal_angle, fg
        scored = [(math.radians(d), self._free(math.radians(d), pts, look))
                  for d in range(-max_deg, max_deg + 1, step_deg)]
        open_ = [(a, f) for a, f in scored if f >= look - 1e-6]
        if open_:
            return min(open_, key=lambda af: abs(af[0] - g))
        return max(scored, key=lambda af: af[1])

    def speed_factor(self, stop=None, slow=None):
        """0 = must stop, 1 = full speed; linear in between."""
        stop = self.stop if stop is None else stop
        slow = self.slow if slow is None else slow
        c = self.clearance()
        if c <= stop:
            return 0.0, c
        if c >= slow:
            return 1.0, c
        return 0.25 + 0.75 * (c - stop) / (slow - stop), c
