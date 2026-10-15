"""Mass moments of solids: OpenCascade's volume properties as additive moments (mass, first and second moments about
the origin), so pieces of a part can be added and subtracted before their densities differ. Shared by libs.cad (the
section build ledger) and libs.computed_geometry (the exact printed mass)."""

from __future__ import annotations

from dataclasses import dataclass

from libs.body_solids import M_TO_MM
from libs.geometry import ExactPiece, Inertia, Vec

# Volume properties come from a triangle mesh of the shape. OpenCascade's surface integration is off by about 7 % on a
# lofted fin at its default (swagger's fused section: 1.44 cm^3 where a fine mesh gives 3.11), and its adaptive mode
# accurate but took minutes per section on the example's fused parts. The mesh is accurate to its deflection and fast.
MESH_LINEAR_DEFLECTION_MM = 0.005
MESH_ANGULAR_DEFLECTION_RAD = 0.05


@dataclass(frozen=True)
class Moments:
    """Mass moments about the origin: mass (kg), first moments (kg m) and raw second moments (kg m^2: sum m xx, yy,
    zz, xy, xz, yz). Additive, so pieces of one part can be added and subtracted before densities differ."""
    mass: float
    first: Vec
    second: tuple[float, float, float, float, float, float]

    def __add__(self, other: Moments) -> Moments:
        return Moments(self.mass + other.mass, tuple(a + b for a, b in zip(self.first, other.first)),
                       tuple(a + b for a, b in zip(self.second, other.second)))

    def __sub__(self, other: Moments) -> Moments:
        return Moments(self.mass - other.mass, tuple(a - b for a, b in zip(self.first, other.first)),
                       tuple(a - b for a, b in zip(self.second, other.second)))

    def scaled(self, factor: float) -> Moments:
        return Moments(self.mass * factor, tuple(value * factor for value in self.first), tuple(value * factor for value in self.second))

    def piece(self, name: str) -> ExactPiece:
        """Mass, centroid and inertia about it per kg (tensor entries, as libs.geometry.Inertia)."""
        if self.mass <= 0.0:
            raise ValueError(f"{name}: the built solids hold no material for it")
        x, y, z = (value / self.mass for value in self.first)
        sxx, syy, szz, sxy, sxz, syz = (value / self.mass for value in self.second)
        cxx, cyy, czz, cxy, cxz, cyz = sxx - x * x, syy - y * y, szz - z * z, sxy - x * y, sxz - x * z, syz - y * z
        inertia: Inertia = (cyy + czz, cxx + czz, cxx + cyy, -cxy, -cxz, -cyz)
        return ExactPiece(self.mass, (x, y, z), inertia)


NO_MOMENTS = Moments(0.0, (0.0, 0.0, 0.0), (0.0,) * 6)


def volume_moments(shape) -> Moments:
    """The shape's volume moments (density 1 kg/m^3: mass = volume in m^3), from OpenCascade's volume properties of its
    triangle mesh (MESH_LINEAR_DEFLECTION_MM)."""
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GProp import GProp_GProps

    if shape is None or not shape.Solids():
        return NO_MOMENTS
    BRepMesh_IncrementalMesh(shape.wrapped, MESH_LINEAR_DEFLECTION_MM, False, MESH_ANGULAR_DEFLECTION_RAD, True)
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props, True, False, True)  # closed shells only, from the triangulation
    volume_mm3 = props.Mass()
    if volume_mm3 <= 0.0:
        return NO_MOMENTS
    centre = props.CentreOfMass()
    centroid = (centre.X() / M_TO_MM, centre.Y() / M_TO_MM, centre.Z() / M_TO_MM)
    matrix = props.MatrixOfInertia()  # about the centroid, tensor entries (products negative), mm^5
    ixx, iyy, izz = matrix.Value(1, 1), matrix.Value(2, 2), matrix.Value(3, 3)
    ixy, ixz, iyz = matrix.Value(1, 2), matrix.Value(1, 3), matrix.Value(2, 3)
    per_volume = 1.0 / volume_mm3 / M_TO_MM ** 2  # mm^5 / mm^3 -> m^2
    covariance = ((iyy + izz - ixx) / 2.0, (ixx + izz - iyy) / 2.0, (ixx + iyy - izz) / 2.0, -ixy, -ixz, -iyz)
    x, y, z = centroid
    products = (x * x, y * y, z * z, x * y, x * z, y * z)
    volume = volume_mm3 / M_TO_MM ** 3
    return Moments(volume, (volume * x, volume * y, volume * z),
                   tuple(volume * (value * per_volume + product) for value, product in zip(covariance, products)))
