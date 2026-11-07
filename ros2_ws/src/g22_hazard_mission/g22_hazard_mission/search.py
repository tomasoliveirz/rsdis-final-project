"""Survey waypoints and a local search using measured values only."""


def lawnmower(x_min, x_max, y_min, y_max, spacing):
    """Waypoints of a back-and-forth route along x, with lanes `spacing` apart in y."""
    if spacing <= 0:
        raise ValueError('lane spacing must be positive')
    route = []
    y, forward = y_min, True
    while y <= y_max + 1e-9:
        xs = (x_min, x_max) if forward else (x_max, x_min)
        route += [(xs[0], y), (xs[1], y)]
        y += spacing
        forward = not forward
    return route


def inside_bounds(x, y, bounds):
    if bounds is None:
        return True
    xmin, xmax, ymin, ymax = bounds
    return xmin <= x <= xmax and ymin <= y <= ymax


class CompassSearch:
    """Finds the strongest reading around a start point.

    Measure the centre, then the 4 points `step` away. Move to the best one if
    it beats the centre, otherwise halve the step. Stop when the step is below
    `min_step` or the probes run out. Points already measured are not visited
    again. `target` is where to measure next and `add()` takes the value there.
    """

    def __init__(self, x, y, step, min_step, max_probes=40, bounds=None):
        if step <= 0 or min_step <= 0 or max_probes < 1:
            raise ValueError('search steps and probe budget must be positive')
        self.bounds = bounds
        if not inside_bounds(x, y, bounds):
            raise ValueError('search starts outside the survey area')
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

    def _choose_centre(self):
        if not self._ring:
            return
        value, x, y = max(self._ring)
        self._ring = []
        if value > self.best:
            self.best, self.x, self.y = value, x, y
        else:
            self.step /= 2

    def _queue_neighbours(self):
        step = self.step
        points = [(self.x + step, self.y), (self.x - step, self.y),
                  (self.x, self.y + step), (self.x, self.y - step)]
        for x, y in points:
            if not inside_bounds(x, y, self.bounds):
                continue
            known = self._measured.get((round(x, 3), round(y, 3)))
            if known is None:
                self._todo.append((x, y))
            else:
                self._ring.append((known, x, y))

    def _next_ring(self):
        while True:
            self._choose_centre()
            if self.step < self.min_step or self.probes_left < 4:
                return
            self._queue_neighbours()
            if self._todo:
                return
            if not self._ring:
                self.step /= 2
