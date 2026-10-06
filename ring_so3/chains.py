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


def _matrix_tuple(matrix: np.ndarray) -> Tuple[Tuple[float, ...], ...]:
    # the link rotations are multiples of 90 deg: store exact 0 and +-1
    cleaned = np.where(np.abs(matrix) < 1e-12, 0.0, matrix)
    cleaned = np.where(np.abs(np.abs(cleaned) - 1.0) < 1e-12, np.sign(cleaned), cleaned)
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


def _registry() -> Dict[str, Chain]:
    chains: Dict[str, Chain] = {}
    for n in range(1, len(GENERIC_AXES) + 1):
        chains["generic%d" % n] = generic_chain(n)
    for n in range(1, len(TIAGO_ARM) + 1):
        chains["tiago%d" % n] = tiago_chain(n)
    return chains


CHAINS: Dict[str, Chain] = _registry()
#: the twelve models of the tests
MODEL_NAMES: List[str] = ["generic%d" % n for n in range(2, 8)] + ["tiago%d" % n for n in range(2, 8)]


def get_chain(name: str) -> Chain:
    try:
        return CHAINS[name]
    except KeyError:
        raise ValueError("unknown chain %r; known: %s" % (name, ", ".join(sorted(CHAINS))))


__all__ = ["CHAINS", "GENERIC_AXES", "MODEL_NAMES", "TIAGO_ARM", "TIAGO_BASE_LINK", "TIAGO_TOOL", "TIAGO_TOOL_LINK",
           "generic_chain", "get_chain", "tiago_chain"]
