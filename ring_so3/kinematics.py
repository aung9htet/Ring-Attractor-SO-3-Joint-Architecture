"""Exact kinematics of a chain of revolute joints: no neurons here.

A :class:`Chain` is a list of :class:`Joint` objects.  Joint ``k`` first applies a
fixed rotation ``A_k`` (how its frame sits in the previous link) and then turns
about one coordinate axis by its angle::

    R_k = R_(k-1) · A_k · Rot(axis_k, q_k)          orientation of link k
    R_ee = R_n · A_tool                             orientation of the end effector

Every entry of these matrices is a sum of products of ``1, cos q_k, sin q_k``
of the joints (the product features of the paper, Appendix E):
:func:`product_features` builds the feature vector ``Φ`` and
:func:`readout_matrix` the fixed matrix ``T`` with ``vec(R) = T Φ``.

Lift, pitch and yaw are the angles the SO-2 code reads from a matrix
(``lift = atan2(r32, r33)``, ``pitch = -asin(r31)``, ``yaw = atan2(r21, r11)``,
Eq. 26 of the paper), which corresponds to ``R = Rz(yaw) Ry(pitch) Rx(lift)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

AXES = ("x", "y", "z")
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
#: average angle between two rotations drawn at random (the level of a readout that knows nothing), deg
CHANCE_ERROR_DEG = float(np.degrees(np.pi / 2.0 + 2.0 / np.pi))


def rot(axis: str, angle: float) -> np.ndarray:
    """Rotation about a coordinate axis."""

    c, s = float(np.cos(angle)), float(np.sin(angle))
    if axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    if axis == "z":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    raise ValueError("unknown axis %r" % (axis,))


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed-axis roll, pitch, yaw as the URDF defines them: ``Rz(yaw) Ry(pitch) Rx(roll)``."""

    return rot("z", yaw) @ rot("y", pitch) @ rot("x", roll)


def other_axes(axis: str) -> Tuple[int, int]:
    """Columns ``(u, v)`` a rotation about ``axis`` mixes: ``col_u' = c col_u + s col_v``, ``col_v' = -s col_u + c col_v``."""

    a = AXES.index(axis)
    return (a + 1) % 3, (a + 2) % 3


@dataclass(frozen=True)
class Joint:
    name: str
    #: coordinate axis the joint turns about, in its own frame
    axis: str
    #: rotation of the joint's frame in the previous link's frame at angle 0
    fixed: Tuple[Tuple[float, ...], ...] = IDENTITY
    #: origin of the joint in the previous link's frame, m (drawing, and translation in SE(3))
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    lower: float = -float(np.pi)
    upper: float = float(np.pi)
    #: True: the angle lives on the whole circle; False: the limits are mapped onto an arc of the ring
    circular: bool = True
    max_velocity: float = 2.0

    def fixed_matrix(self) -> np.ndarray:
        return np.array(self.fixed, dtype=float)


@dataclass(frozen=True)
class Chain:
    name: str
    joints: Tuple[Joint, ...]
    tool_fixed: Tuple[Tuple[float, ...], ...] = IDENTITY
    tool_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    description: str = ""

    @property
    def n(self) -> int:
        return len(self.joints)

    def link_rotations(self, q: Sequence[float]) -> List[np.ndarray]:
        """Orientation of every link, ``R_1 .. R_n``."""

        angles = self._angles(q)
        rotation, out = np.eye(3), []
        for joint, angle in zip(self.joints, angles):
            rotation = rotation @ joint.fixed_matrix() @ rot(joint.axis, angle)
            out.append(rotation)
        return out

    def rotation(self, q: Sequence[float]) -> np.ndarray:
        """Orientation of the end effector."""

        return self.link_rotations(q)[-1] @ np.array(self.tool_fixed, dtype=float)

    def frames(self, q: Sequence[float]) -> List[Tuple[np.ndarray, np.ndarray]]:
        """``(origin, orientation)`` of the base, of every link and of the tool, in the base frame."""

        angles = self._angles(q)
        origin, rotation = np.zeros(3), np.eye(3)
        out = [(origin.copy(), rotation.copy())]
        for joint, angle in zip(self.joints, angles):
            origin = origin + rotation @ np.array(joint.offset, dtype=float)
            rotation = rotation @ joint.fixed_matrix() @ rot(joint.axis, angle)
            out.append((origin.copy(), rotation.copy()))
        origin = origin + rotation @ np.array(self.tool_offset, dtype=float)
        out.append((origin, rotation @ np.array(self.tool_fixed, dtype=float)))
        return out

    def prefix(self, n: int, name: Optional[str] = None) -> "Chain":
        """The first ``n`` joints; the end effector is then link ``n`` itself."""

        if not 1 <= int(n) <= self.n:
            raise ValueError("%s has %d joints, asked for %d" % (self.name, self.n, n))
        if int(n) == self.n:
            return self
        return Chain(name or "%s_%d" % (self.name, n), self.joints[: int(n)],
                     description="first %d joints of %s" % (n, self.name))

    def limits(self) -> np.ndarray:
        return np.array([[j.lower, j.upper] for j in self.joints], dtype=float)

    def clip(self, q: Sequence[float]) -> np.ndarray:
        limits = self.limits()
        return np.clip(np.asarray(q, dtype=float), limits[:, 0], limits[:, 1])

    def random_pose(self, rng: np.random.Generator) -> np.ndarray:
        limits = self.limits()
        return rng.uniform(limits[:, 0], limits[:, 1])

    def _angles(self, q: Sequence[float]) -> np.ndarray:
        angles = np.asarray(q, dtype=float).reshape(-1)
        if angles.size != self.n:
            raise ValueError("%s has %d joints, got %d angles" % (self.name, self.n, angles.size))
        return angles


# -- angles and errors -------------------------------------------------------
def lift_pitch_yaw(rotation: np.ndarray) -> Tuple[float, float, float]:
    r = np.asarray(rotation, dtype=float)
    pitch = -np.arcsin(np.clip(r[2, 0], -1.0, 1.0))
    lift = np.arctan2(r[2, 1], r[2, 2])
    yaw = np.arctan2(r[1, 0], r[0, 0])
    return float(lift), float(pitch), float(yaw)


def from_lift_pitch_yaw(lift: float, pitch: float, yaw: float) -> np.ndarray:
    return rot("z", yaw) @ rot("y", pitch) @ rot("x", lift)


def nearest_rotation(matrix: np.ndarray) -> np.ndarray:
    """The proper rotation closest to a 3 x 3 matrix (least squares, by SVD)."""

    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=float))
    d = np.sign(np.linalg.det(u @ vt)) or 1.0
    return u @ np.diag([1.0, 1.0, d]) @ vt


def rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Angle (rad) of the single rotation that turns orientation ``a`` into ``b``."""

    relative = np.asarray(a, dtype=float).T @ np.asarray(b, dtype=float)
    return float(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))


def orientation_error_deg(estimate: np.ndarray, true: np.ndarray) -> float:
    """Orientation error in degrees; ``estimate`` may be a raw readout, it is first made a proper rotation."""

    matrix = np.asarray(estimate, dtype=float)
    if not np.all(np.isfinite(matrix)):
        return float("nan")
    return float(np.degrees(rotation_angle(nearest_rotation(matrix), true)))


def orthonormality_defect(matrix: np.ndarray) -> float:
    """How far a raw 3 x 3 readout is from a rotation: Frobenius norm of ``MᵀM - I``."""

    m = np.asarray(matrix, dtype=float)
    return float(np.linalg.norm(m.T @ m - np.eye(3)))


def circular_difference(a: float, b: float) -> float:
    """``a - b`` on the circle, in ``(-pi, pi]``."""

    return float(np.angle(np.exp(1j * (float(a) - float(b)))))


# -- the product features of the paper ---------------------------------------
Monomial = Tuple[int, ...]          # per joint: 0 -> 1, 1 -> cos, 2 -> sin
Polynomial = Dict[Monomial, float]


def joint_features(q: Sequence[float]) -> np.ndarray:
    """``(n, 3)`` array of ``1, cos q_k, sin q_k``."""

    angles = np.asarray(q, dtype=float).reshape(-1)
    return np.stack([np.ones_like(angles), np.cos(angles), np.sin(angles)], axis=1)


def product_features(q: Sequence[float]) -> np.ndarray:
    """``Φ = φ_1 ⊗ φ_2 ⊗ ... ⊗ φ_n`` (``3^n`` values; joint 1 is the slowest index)."""

    phi = np.ones(1)
    for row in joint_features(q):
        phi = np.kron(phi, row)
    return phi


def monomial_index(monomial: Monomial) -> int:
    index = 0
    for part in monomial:
        index = index * 3 + int(part)
    return index


def _poly_mul(a: Polynomial, b: Polynomial) -> Polynomial:
    out: Polynomial = {}
    for ma, ca in a.items():
        for mb, cb in b.items():
            key = tuple(x + y for x, y in zip(ma, mb))     # the factors belong to different joints
            out[key] = out.get(key, 0.0) + ca * cb
    return {m: c for m, c in out.items() if abs(c) > 1e-12}


def _poly_add(terms: Iterable[Polynomial]) -> Polynomial:
    out: Polynomial = {}
    for term in terms:
        for m, c in term.items():
            out[m] = out.get(m, 0.0) + c
    return {m: c for m, c in out.items() if abs(c) > 1e-12}


def _poly_matmul(a, b):
    return [[_poly_add(_poly_mul(a[i][k], b[k][j]) for k in range(3)) for j in range(3)] for i in range(3)]


def _poly_constant(matrix: np.ndarray, n: int):
    one = tuple([0] * n)
    return [[({one: float(matrix[i][j])} if abs(matrix[i][j]) > 1e-12 else {}) for j in range(3)] for i in range(3)]


def _poly_joint(axis: str, k: int, n: int):
    def mono(part: int) -> Monomial:
        m = [0] * n
        m[k] = part
        return tuple(m)

    one, c, s, ms = {tuple([0] * n): 1.0}, {mono(1): 1.0}, {mono(2): 1.0}, {mono(2): -1.0}
    z: Polynomial = {}
    if axis == "x":
        return [[one, z, z], [z, c, ms], [z, s, c]]
    if axis == "y":
        return [[c, z, s], [z, one, z], [ms, z, c]]
    return [[c, ms, z], [s, c, z], [z, z, one]]


def rotation_polynomials(chain: Chain):
    """The end-effector matrix with every entry written as a sum of products of ``1, cos, sin``."""

    n = chain.n
    matrix = _poly_constant(np.eye(3), n)
    for k, joint in enumerate(chain.joints):
        matrix = _poly_matmul(matrix, _poly_constant(joint.fixed_matrix(), n))
        matrix = _poly_matmul(matrix, _poly_joint(joint.axis, k, n))
    return _poly_matmul(matrix, _poly_constant(np.array(chain.tool_fixed, dtype=float), n))


def readout_matrix(chain: Chain) -> np.ndarray:
    """``T`` (9 x 3^n) of the paper's Eq. 22: ``R.reshape(9) = T @ product_features(q)``."""

    polys = rotation_polynomials(chain)
    t = np.zeros((9, 3 ** chain.n))
    for i in range(3):
        for j in range(3):
            for monomial, coefficient in polys[i][j].items():
                t[3 * i + j, monomial_index(monomial)] = coefficient
    return t


def used_products(chain: Chain) -> List[Monomial]:
    """The products of ``cos`` and ``sin`` that occur in the end-effector matrix (without the constant)."""

    polys = rotation_polynomials(chain)
    found = {m for row in polys for entry in row for m in entry if any(m)}
    return sorted(found)


def monomial_name(monomial: Monomial) -> str:
    return "·".join("%s%d" % ("cs"[part - 1], k + 1) for k, part in enumerate(monomial) if part) or "1"


__all__ = [
    "AXES", "CHANCE_ERROR_DEG", "Chain", "IDENTITY", "Joint", "circular_difference", "from_lift_pitch_yaw",
    "joint_features", "lift_pitch_yaw", "monomial_index", "monomial_name", "nearest_rotation",
    "orientation_error_deg", "orthonormality_defect", "other_axes", "product_features", "readout_matrix",
    "rot", "rotation_angle", "rotation_polynomials", "rpy_matrix", "used_products",
]
