import numpy as np
from magpylib.magnet import Cuboid, Cylinder, Sphere, Tetrahedron, TriangularMesh
from magpylib.misc import Dipole, Triangle
from scipy.spatial.transform import Rotation

POSITION = (0.1, 0.2, 0.3)
ORIENTATION = Rotation.from_rotvec([np.pi / 7, np.pi / 6, np.pi / 5])

def get_points():
    bounds = np.array([[-0.5, 0.5]] * 3)
    N = [10] * 3
    linsp = [np.linspace(bounds[i, 0], bounds[i, 1], n) for i, n in enumerate(N)]
    mesh = np.meshgrid(*linsp)
    return np.column_stack([m.flatten() for m in mesh])

class CylinderMagnetBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Cylinder(
            position=POSITION,
            orientation=ORIENTATION,
            dimension=np.array((0.1, 0.2)),
            polarization=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)

class CuboidMagnetBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Cuboid(
            position=POSITION,
            orientation=ORIENTATION,
            dimension=np.array((0.1, 0.2, 0.3)),
            polarization=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)

class DipoleBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Dipole(
            position=POSITION,
            orientation=ORIENTATION,
            moment=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)

class SphereMagnetBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Sphere(
            position=POSITION,
            orientation=ORIENTATION,
            diameter=0.1,
            polarization=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)

class TriangleMagnetBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Triangle(
            position=POSITION,
            orientation=ORIENTATION,
            vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, 0.1], [0.0, 0.2, 0.0]]),
            polarization=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)

class TetrahedronMagnetBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = Tetrahedron(
            position=POSITION,
            orientation=ORIENTATION,
            vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]]),
            polarization=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)

class MeshMagnetBenchmark:
    def setup(self):
        self.points = get_points()
        self.source = TriangularMesh(
            position=POSITION,
            orientation=ORIENTATION,
            vertices=np.array([[-0.1, -0.1, -0.1], [0.1, -0.1, -0.1], [0.0, 0.1, -0.1], [0.0, 0.0, 0.1]]),
            faces=np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]),
            polarization=np.array((1.0, 2.0, 3.0))
        )
    def time_getB(self):
        self.source.getB(self.points)
