"""Flight patterns for the UAV. No ROS in here, so it can be tested with pytest."""


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
