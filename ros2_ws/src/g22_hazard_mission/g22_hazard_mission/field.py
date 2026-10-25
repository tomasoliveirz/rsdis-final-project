"""Gas and radiation values at any point, from the sources in config/hazards.yaml.

Gas: each leak is a Gaussian that gets weaker with distance and with height,
because the gas stays close to the ground.
    gas = strength * exp(-r^2 / (2 spread^2)) * exp(-z^2 / (2 height^2))

Radiation: inverse square law. We add 1 m^2 so the value doesn't blow up right
on top of the source.
    rad = strength / (1 + r^2 + z^2)

r is the horizontal distance to the source and z the height. Both values get a
background level and some noise. No ROS in here, so it can be tested with pytest.
"""
from dataclasses import dataclass
import math
import random

import yaml


@dataclass
class Source:
    name: str
    type: str       # 'gas' or 'radiation'
    x: float
    y: float
    strength: float  # gas: peak ppm at the leak, radiation: uSv/h at 1 m
    spread: float = 2.5  # gas only, metres
    height: float = 6.0  # gas only, metres


@dataclass
class Field:
    sources: list
    gas_background: float = 0.5
    radiation_background: float = 0.1
    noise: float = 0.05  # relative noise of each reading

    def gas(self, x, y, z):
        c = self.gas_background
        for s in self.sources:
            if s.type == 'gas':
                r2 = (x - s.x) ** 2 + (y - s.y) ** 2
                c += s.strength * math.exp(-r2 / (2 * s.spread ** 2)) \
                    * math.exp(-z ** 2 / (2 * s.height ** 2))
        return c

    def radiation(self, x, y, z):
        d = self.radiation_background
        for s in self.sources:
            if s.type == 'radiation':
                d += s.strength / (1.0 + (x - s.x) ** 2 + (y - s.y) ** 2 + z ** 2)
        return d

    def measure(self, x, y, z, rng=random):
        """Noisy gas (ppm) and radiation (uSv/h) reading at (x, y, z)."""
        gas = self.gas(x, y, z)
        rad = self.radiation(x, y, z)
        gas = max(0.0, rng.gauss(gas, self.noise * gas))
        rad = max(0.0, rng.gauss(rad, self.noise * rad))
        return gas, rad


def load(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    bg = data.get('background', {})
    return Field(
        sources=[Source(**s) for s in data.get('sources', [])],
        gas_background=bg.get('gas_ppm', 0.5),
        radiation_background=bg.get('radiation_usvh', 0.1),
        noise=data.get('noise', 0.05),
    )
