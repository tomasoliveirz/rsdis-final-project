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
    """Measure the centre and four neighbours, at most twice (nine probes).

    Keep the strongest measured point. Stop after a round without improvement.
    There is no field model, fitting, step refinement or cache of old readings.
    """

    def __init__(self, x, y, step=2.0, rounds=2, bounds=None):
        if step <= 0 or rounds < 1:
            raise ValueError('search step and round count must be positive')
        if not inside_bounds(x, y, bounds):
            raise ValueError('search starts outside the survey area')
        self.x, self.y = x, y
        self.step, self.rounds, self.bounds = step, rounds, bounds
        self.best = None
        self._todo = [(x, y)]
        self._round = []

    @property
    def done(self):
        return not self._todo

    @property
    def target(self):
        return self._todo[0]

    def add(self, value):
        x, y = self._todo.pop(0)
        self._round.append((value, x, y))
        if self._todo:
            return
        value, x, y = max(self._round)
        self._round = []
        if self.best is not None:
            self.rounds -= 1
            if value <= self.best:
                return
        self.best, self.x, self.y = value, x, y
        if self.rounds:
            s = self.step
            self._todo = [(x + s, y), (x, y + s), (x - s, y), (x, y - s)]
            self._todo = [p for p in self._todo if inside_bounds(*p, self.bounds)]
