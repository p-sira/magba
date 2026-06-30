import numpy as np
from magpylib.current import Circle, TriangleSheet, Polyline
from scipy.spatial.transform import Rotation

POSITION = (0.1, 0.2, 0.3)
ORIENTATION = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])

def get_points():
    bounds = np.array([[-0.5, 0.5]] * 3)
    N = [10] * 3
    linsp = [np.linspace(bounds[i, 0], bounds[i, 1], n) for i, n in enumerate(N)]
    mesh = np.meshgrid(*linsp)
    return np.column_stack([m.flatten() for m in mesh])

class CircularCurrentBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Circle(
            position=POSITION,
            orientation=ORIENTATION,
            diameter=1.0,
            current=1.0
        )
    
    def time_getB(self):
        self.source.getB(self.points)

class PathCurrentBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Polyline(
            position=POSITION,
            orientation=ORIENTATION,
            current=100.0,
            vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]])
        )

    def time_getB(self):
        self.source.getB(self.points)

class SheetCurrentBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = TriangleSheet(
            position=POSITION,
            orientation=ORIENTATION,
            vertices=[
                (-0.1, -0.1, -0.1),
                (0.1, -0.1, -0.1),
                (0.0, 0.1, -0.1),
                (0.0, 0.0, 0.1),
            ],
            faces=[(0, 2, 1), (0, 1, 3), (1, 2, 3), (0, 3, 2)],
            current_densities=[
                (1.0, 2.0, 3.0),
                (1.0, 2.0, 3.0),
                (1.0, 2.0, 3.0),
                (1.0, 2.0, 3.0),
            ]
        )
    
    def time_getB(self):
        self.source.getB(self.points)

class TriangleCurrentBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = TriangleSheet(
            position=POSITION,
            orientation=ORIENTATION,
            vertices=[
                (-0.1, -0.1, -0.1),
                (0.1, -0.1, -0.1),
                (0.0, 0.1, -0.1),
            ],
            faces=[[0, 1, 2]],
            current_densities=[(1.0, 2.0, 3.0)]
        )

    def time_getB(self):
        self.source.getB(self.points)
