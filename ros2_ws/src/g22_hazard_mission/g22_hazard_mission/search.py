"""Flight patterns and source estimation for the UAV. No ROS in here, so it can be tested with pytest."""
import math

import numpy as np


def lawnmower(x_min, x_max, y_min, y_max, spacing):
    """Waypoints of a back-and-forth route along x, with lanes `spacing` apart in y."""
    route = []
    y, forward = y_min, True
    while y <= y_max + 1e-9:
        xs = (x_min, x_max) if forward else (x_max, x_min)
        route += [(xs[0], y), (xs[1], y)]
        y += spacing
        forward = not forward
    return route


class CompassSearch:
    """Finds the strongest reading around a start point.

    Measure the centre, then the 4 points `step` away. Move to the best one if
    it beats the centre, otherwise halve the step. Stop when the step is below
    `min_step` or the probes run out. Points already measured are not visited
    again. `target` is where to measure next and `add()` takes the value there.
    """

    def __init__(self, x, y, step, min_step, max_probes=40):
        self.x, self.y = x, y
        self.best = None
        self.step = step
        self.min_step = min_step
        self.probes_left = max_probes
        self._todo = [(x, y)]
        self._ring = []
        self._measured = {}

    @property
    def done(self):
        return not self._todo

    @property
    def target(self):
        return self._todo[0]

    def add(self, value):
        px, py = self._todo.pop(0)
        self.probes_left -= 1
        self._measured[(round(px, 3), round(py, 3))] = value
        if self.best is None:
            self.best = value
        else:
            self._ring.append((value, px, py))
        if not self._todo:
            self._next_ring()

    def _next_ring(self):
        while True:
            if self._ring:
                v, px, py = max(self._ring)
                self._ring = []
                if v > self.best:
                    self.best, self.x, self.y = v, px, py
                else:
                    self.step /= 2
            if self.step < self.min_step or self.probes_left < 4:
                return
            s = self.step
            for p in [(self.x + s, self.y), (self.x - s, self.y),
                      (self.x, self.y + s), (self.x, self.y - s)]:
                known = self._measured.get((round(p[0], 3), round(p[1], 3)))
                if known is None:
                    self._todo.append(p)
                else:
                    self._ring.append((known, *p))
            if self._todo:
                return
def grid(cx, cy, step):
    """3 x 3 points around (cx, cy), in snake order so the UAV does not cross back."""
    return [(cx + i * step, cy + j * step)
            for i in (-1, 0, 1) for j in ((-1, 0, 1) if i != 0 else (1, 0, -1))]


def estimate_source(samples, hazard_type, background):
    """Position of the source from (x, y, value) readings, or None.

    Both sensor models become a paraboloid in x and y after a transform:
      gas (Gaussian):          ln(value - bg) = a + b x + c y + d (x^2 + y^2), d < 0
      radiation (1/r^2 law):   1/(value - bg) = a + b x + c y + d (x^2 + y^2), d > 0
    A least-squares fit gives a, b, c, d and the source is at (-b / 2d, -c / 2d).
    """
    rows, ys = [], []
    for x, y, v in samples:
        excess = v - background
        if excess <= 1e-3:
            continue
        rows.append([1.0, x, y, x * x + y * y])
        ys.append(math.log(excess) if hazard_type == 'gas' else 1.0 / excess)
    if len(rows) < 4:
        return None
    a, b, c, d = np.linalg.lstsq(np.array(rows), np.array(ys), rcond=None)[0]
    if (hazard_type == 'gas' and d >= 0) or (hazard_type == 'radiation' and d <= 0):
        return None  # no peak in these readings
    return -b / (2 * d), -c / (2 * d)
