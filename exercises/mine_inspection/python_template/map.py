import math

import numpy as np

MAP_SIZE_PX = 500
MAP_EXTENT_M = 40.0


def world_to_grid(x, y):
    scale = MAP_SIZE_PX / MAP_EXTENT_M
    cx = MAP_SIZE_PX // 2
    cy = MAP_SIZE_PX // 2
    gx = int(cx + x * scale)
    gy = int(cy - y * scale)
    return gx, gy


class Map:
    """Accumulate an occupancy grid and the rover path from laser and pose."""

    def __init__(self, laser_callback, pose_callback):
        self.laser_callback = laser_callback
        self.pose_callback = pose_callback

        self.occupancy = np.full((MAP_SIZE_PX, MAP_SIZE_PX, 3), 240, dtype=np.uint8)
        self.path_cells = []

        self.ch4 = 0.0
        self.co = 0.0
        self.crack = False

    def set_readings(self, ch4, co, crack):
        self.ch4 = ch4
        self.co = co
        self.crack = crack

    def update(self):
        pose = self.pose_callback()
        if pose is None:
            return
        laser = self.laser_callback()
        if laser is None or len(laser.values) == 0:
            return

        gx, gy = world_to_grid(pose.x, pose.y)
        if 0 <= gx < MAP_SIZE_PX and 0 <= gy < MAP_SIZE_PX:
            self.path_cells.append((gx, gy))

        n = len(laser.values)
        if n == 0:
            return
        angle = laser.minAngle
        dtheta = (laser.maxAngle - laser.minAngle) / max(n - 1, 1)
        max_range = laser.maxRange if laser.maxRange > 0 else 10.0

        for i in range(n):
            d = laser.values[i]
            theta = angle + i * dtheta
            if d == float("inf") or d <= 0 or d > max_range:
                angle = angle
                continue
            obstacle_x = pose.x + d * math.cos(theta + pose.yaw - math.pi / 2)
            obstacle_y = pose.y + d * math.sin(theta + pose.yaw - math.pi / 2)
            ox, oy = world_to_grid(obstacle_x, obstacle_y)
            if 0 <= ox < MAP_SIZE_PX and 0 <= oy < MAP_SIZE_PX:
                self.occupancy[oy, ox] = (30, 30, 30)

    def render(self):
        image = self.occupancy.copy()
        for gx, gy in self.path_cells[-1000:]:
            if 0 <= gx < MAP_SIZE_PX and 0 <= gy < MAP_SIZE_PX:
                image[gy, gx] = (80, 120, 200)

        pose = self.pose_callback()
        if pose is not None:
            gx, gy = world_to_grid(pose.x, pose.y)
            for dx in range(-4, 5):
                for dy in range(-4, 5):
                    px = gx + dx
                    py = gy + dy
                    if (
                        0 <= px < MAP_SIZE_PX
                        and 0 <= py < MAP_SIZE_PX
                        and dx * dx + dy * dy <= 16
                    ):
                        image[py, px] = (30, 220, 80)
        return image

    def reset(self):
        self.occupancy = np.full((MAP_SIZE_PX, MAP_SIZE_PX, 3), 240, dtype=np.uint8)
        self.path_cells = []
        self.ch4 = 0.0
        self.co = 0.0
        self.crack = False
