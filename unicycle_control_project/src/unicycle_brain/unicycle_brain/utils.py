from enum import Enum

class trajectory_type(Enum):
    point = 0
    Lissajou_curves = 1
    curve = 2
    harmonic = 3
    square = 4
    saw = 5
    not_continuous = 6

