"""The chains the decoder is built for.  A new chain is one entry here.

* ``generic2`` .. ``generic7``: joints about x, y, z, x, y, z, x with no fixed
  rotation between them and angles on the whole circle.  ``generic2`` is the
  chain of the paper, ``R = Rx(q1) Ry(q2)``.
* ``tiago2`` .. ``tiago7``: the first joints of the TIAGo arm (``arm_1_joint`` ..
  ``arm_7_joint``) in the frame of ``torso_lift_link``.  Axes, link rotations,
  offsets, limits and velocity limits are the values of the robot's URDF
  (``tiago_description``, arm with pal-hey5 hand and schunk-ft sensor); every
  joint turns about its local z and the link rotations are multiples of 90 deg.
  ``tiago7`` ends at ``arm_tool_link``.  ``tests/test_kinematics.py`` compares
  this table with the URDF itself when the robot description is installed.
* ``kangaroo_left4``, ``kangaroo_right4``: the four joints of PAL's Kangaroo arms
  (``arm_<side>_1_joint`` .. ``arm_<side>_4_joint``) in the frame of ``torso_link``,
  ending at ``arm_<side>_tool_link``; see :func:`kangaroo_chain`.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from .kinematics import Chain, Joint, rpy_matrix

GENERIC_AXES = ("x", "y", "z", "x", "y", "z", "x")
GENERIC_LINK_M = 0.2
H = float(np.pi / 2.0)

#: name, origin xyz (m), origin rpy (rad), lower, upper (rad), velocity limit (rad/s)
TIAGO_ARM = (
    ("arm_1_joint", (0.15505, 0.014, -0.151), (0.0, 0.0, -H), 0.0, 2.748893571891069, 1.95),
    ("arm_2_joint", (0.125, 0.0195, -0.031), (H, 0.0, 0.0), -1.5707963267948966, 1.090830782496456, 1.95),
    ("arm_3_joint", (0.0895, 0.0, -0.0015), (-H, 0.0, H), -3.5342917352885173, 1.5707963267948966, 2.35),
    ("arm_4_joint", (-0.02, -0.027, -0.222), (-H, -H, 0.0), -0.39269908169872414, 2.356194490192345, 2.35),
    ("arm_5_joint", (-0.162, 0.02, 0.027), (H, -H, -H), -2.0943951023931953, 2.0943951023931953, 1.95),
    ("arm_6_joint", (0.0, 0.0, 0.15), (0.0, -H, -H), -1.413716694115407, 1.413716694115407, 1.76),
    ("arm_7_joint", (0.0, 0.0, 0.0), (H, 0.0, H), -2.0943951023931953, 2.0943951023931953, 1.76),
)
TIAGO_TOOL = ("arm_tool_joint", (0.0, 0.0, 0.046), (H, -H, float(np.pi)))
TIAGO_BASE_LINK = "torso_lift_link"
TIAGO_TOOL_LINK = "arm_tool_link"

_KANGAROO_JOINTS = (   # name, xyz, rpy, axis sign, lower, upper, velocity limit (the URDF of the robot)
    ("arm_%s_1_joint", (0.0, 0.0, 0.0), (3.1416, 0.0, 3.1416), -1, -2.6179938779914944, 2.6179938779914944, 1.95),
    ("arm_%s_2_joint", (0.03711, 0.02, -0.17011), (0.0, 1.5708, 0.0), 1, -1.1344640137963142, None, 1.95),
    ("arm_%s_3_joint", (0.15989, -0.02, -0.03711), (0.0, -1.5708, 0.0), -1, -2.6179938779914944, 2.6179938779914944, 2.35),
    ("arm_%s_4_joint", (-0.03711, -0.02, -0.17011), (3.14159, 1.5708, 0.0), 1, -1.1345, 2.4435, 2.35),
)
#: upper limit of arm_<side>_2_joint measured by SO-2 (narrower than the URDF's 2.443460952792061):
#: ../Ring-Attractor-SO-2-Joint-Architecture/src/kangaroo_ring_controller/results/joint_limits/
#: left_j1_20261013_180318 (left), right_j1_20261013_125903 (right)
_KANGAROO_J2_UPPER = {"left": 1.4229904547193268, "right": 1.4622630704048099}
#: per side, rows ``(name, xyz, rpy, axis_sign, lower, upper, velocity)`` of joints 1 .. 4
KANGAROO_ARM = {
    side: tuple((n % side, xyz, rpy, sign, lo, _KANGAROO_J2_UPPER[side] if up is None else up, v)
                for n, xyz, rpy, sign, lo, up, v in _KANGAROO_JOINTS)
    for side in ("left", "right")
}
#: arm_<side>_fixed_joint, torso_link -> arm_<side>_base_link: xyz, rpy
KANGAROO_MOUNT = {
    "left": ((0.0, 0.105954, 0.384396), (1.3963, 3.14159, 3.14159)),
    "right": ((0.0, -0.105954, 0.384396), (1.3963, 3.14159, 0.0)),
}
KANGAROO_TOOL = ((0.35, -0.02, -0.0255), (0.0, 0.0, 0.0))     # arm_<side>_4_link -> arm_<side>_tool_link
KANGAROO_BASE_LINK = "torso_link"
KANGAROO_TOOL_LINK = "arm_%s_tool_link"


def _matrix_tuple(matrix: np.ndarray, tol: float = 1e-12) -> Tuple[Tuple[float, ...], ...]:
    # the link rotations are multiples of 90 deg: store exact 0 and +-1
    cleaned = np.where(np.abs(matrix) < tol, 0.0, matrix)
    cleaned = np.where(np.abs(np.abs(cleaned) - 1.0) < tol, np.sign(cleaned), cleaned)
    return tuple(tuple(float(v) for v in row) for row in cleaned)


def generic_chain(n: int) -> Chain:
    if not 1 <= int(n) <= len(GENERIC_AXES):
        raise ValueError("generic chains have 1 to %d joints" % len(GENERIC_AXES))
    joints = tuple(
        Joint("q%d" % (k + 1), GENERIC_AXES[k], offset=(0.0, 0.0, 0.0 if k == 0 else GENERIC_LINK_M))
        for k in range(int(n))
    )
    return Chain("generic%d" % n, joints, tool_offset=(0.0, 0.0, GENERIC_LINK_M),
                 description="joints about %s, angles on the whole circle" % ", ".join(GENERIC_AXES[: int(n)]))


def tiago_chain(n: int = 7) -> Chain:
    if not 1 <= int(n) <= len(TIAGO_ARM):
        raise ValueError("the TIAGo arm has %d joints" % len(TIAGO_ARM))
    joints = tuple(
        Joint(name, "z", fixed=_matrix_tuple(rpy_matrix(*rpy)), offset=tuple(xyz), lower=lower, upper=upper,
              circular=False, max_velocity=velocity)
        for name, xyz, rpy, lower, upper, velocity in TIAGO_ARM[: int(n)]
    )
    if int(n) == len(TIAGO_ARM):
        _, xyz, rpy = TIAGO_TOOL
        return Chain("tiago7", joints, tool_fixed=_matrix_tuple(rpy_matrix(*rpy)), tool_offset=tuple(xyz),
                     description="TIAGo arm, arm_1 .. arm_7, ending at arm_tool_link")
    return Chain("tiago%d" % n, joints, description="TIAGo arm, arm_1 .. arm_%d" % n)


def kangaroo_chain(side: str) -> Chain:
    """The four joints of one Kangaroo arm, ``torso_link`` to ``arm_<side>_tool_link``.

    ``Joint.axis`` is x, y or z, so the URDF axes -z of joints 1 and 3 are folded:
    ``Rot(-z, q) = Rx(pi) Rz(q) Rx(pi)``.  Joint angles equal the robot's; links 2, 4 and the
    tool frame equal the URDF's; links 1 and 3 are the URDF frames turned half a turn about
    their x axis.
    """

    if side not in KANGAROO_ARM:
        raise ValueError("side must be 'left' or 'right', got %r" % (side,))
    flip, carry, joints = rpy_matrix(np.pi, 0.0, 0.0), np.eye(3), []
    for k, (name, xyz, rpy, sign, lower, upper, velocity) in enumerate(KANGAROO_ARM[side]):
        mount_xyz, mount_rpy = KANGAROO_MOUNT[side]
        fixed = (rpy_matrix(*mount_rpy) if k == 0 else carry) @ rpy_matrix(*rpy)
        offset = mount_xyz if k == 0 else carry @ np.array(xyz)
        carry = flip if sign < 0 else np.eye(3)
        # the URDF writes pi/2 and pi as 1.5708, 3.1416, 3.14159; composed, the rounding leaves residuals up to
        # ~1.2e-5: snap entries within 2e-5 of 0, +-1 (<= 0.0012 deg) so the wiring gets no such tiny products
        joints.append(Joint(name, "z", fixed=_matrix_tuple(fixed @ carry, 2e-5), offset=tuple(float(v) for v in offset),
                            lower=lower, upper=upper, circular=False, max_velocity=velocity))
    xyz, rpy = KANGAROO_TOOL
    return Chain("kangaroo_%s4" % side, tuple(joints), tool_fixed=_matrix_tuple(carry @ rpy_matrix(*rpy), 2e-5),
                 tool_offset=tuple(float(v) for v in carry @ np.array(xyz)),
                 description="Kangaroo %s arm, arm_%s_1 .. arm_%s_4, ending at arm_%s_tool_link" % ((side,) * 4))


def _registry() -> Dict[str, Chain]:
    chains: Dict[str, Chain] = {}
    for n in range(1, len(GENERIC_AXES) + 1):
        chains["generic%d" % n] = generic_chain(n)
    for n in range(1, len(TIAGO_ARM) + 1):
        chains["tiago%d" % n] = tiago_chain(n)
    for side in KANGAROO_ARM:
        chains["kangaroo_%s4" % side] = kangaroo_chain(side)
    return chains


CHAINS: Dict[str, Chain] = _registry()
#: the twelve models of the tests
MODEL_NAMES: List[str] = ["generic%d" % n for n in range(2, 8)] + ["tiago%d" % n for n in range(2, 8)]

KANGAROO_MODELS: List[str] = ["kangaroo_left4", "kangaroo_right4"]


def get_chain(name: str) -> Chain:
    try:
        return CHAINS[name]
    except KeyError:
        raise ValueError("unknown chain %r; known: %s" % (name, ", ".join(sorted(CHAINS))))


__all__ = ["CHAINS", "GENERIC_AXES", "KANGAROO_ARM", "KANGAROO_BASE_LINK", "KANGAROO_MODELS", "KANGAROO_MOUNT",
           "KANGAROO_TOOL", "KANGAROO_TOOL_LINK", "MODEL_NAMES", "TIAGO_ARM", "TIAGO_BASE_LINK", "TIAGO_TOOL",
           "TIAGO_TOOL_LINK", "generic_chain", "get_chain", "kangaroo_chain", "tiago_chain"]
