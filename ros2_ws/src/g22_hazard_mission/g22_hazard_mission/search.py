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
