"""The comparison and the cascade as small steps on top of the forward wiring.

The forward wiring (:mod:`ring_so3.wiring`) gives every entry of every link's orientation
as a combination of nodes.  :func:`ik_wiring` adds, as a list of steps the spiking network
and the exact evaluation both follow:

* ``("gfeat", c, part)``: cos (part 0) and sin (part 1) of goal ring ``c``
  (0: yaw, 1: pitch, 2: lift, the order of :data:`ring_so3.ik.rules.GOAL_CHAIN`);
* ``("g", node)``: the nodes of the goal chain (comparison ``matrix``): the goal matrix
  built joint by joint from the three goal rings, with the forward chain's own wiring;
* ``("ikmul", j)``: a product of two earlier values;
* ``("adiff", c)``: comparison ``angles``: the sine-difference of lift, pitch, yaw;
* ``("turn", i)``: the comparison ``e``, times :data:`TURN_SCALE`;
* ``("share", k, +1)`` and ``("share", k, -1)``: the two halves of joint k's share, each
  between 0 and 1, in units of the joint's cap: share = cap_k · (plus − minus);
* ``("rest", k, i)``: what is left after joint k (times :data:`TURN_SCALE`); ``k = n`` is
  what the whole arm could not take.

Constants are folded: joint 1 turns about a fixed axis, so its share and what it leaves
need no product.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from ..kinematics import AXES, Chain
from ..wiring import ONE, Combo, Entry, Node, Product, Wiring, chain_wiring
from . import rules
from .design import IkDesign

Terms = Tuple[Tuple[Node, float], ...]
EPS = 1e-12
#: the comparison is held as ``TURN_SCALE · e``: a turn of 0.5 rad fills the range of a population
TURN_SCALE = 2.0
NORM: Node = ("norm",)            # cos(pitch) of the decoded orientation: the length population of the forward network


@dataclass(frozen=True)
class Mul:
    index: int
    a: Terms
    b: Terms
    #: a product of the goal chain keeps that chain's node name
    name: Optional[Node] = None

    @property
    def node(self) -> Node:
        return self.name if self.name is not None else ("ikmul", self.index)


@dataclass(frozen=True)
class Relay:
    """A population that holds a weighted sum of earlier values (clipped to [-1, 1])."""

    node: Node
    terms: Terms


@dataclass(frozen=True)
class Half:
    """A population that holds the positive part of a weighted sum, clipped to [0, 1]."""

    node: Node
    terms: Terms


IkStep = Union[Mul, Relay, Half]


def _clean(combo: Combo) -> Combo:
    return {node: float(c) for node, c in combo.items() if abs(c) >= EPS}


def _sum(combos: Sequence[Combo], weights: Optional[Sequence[float]] = None) -> Combo:
    out: Combo = {}
    for index, combo in enumerate(combos):
        weight = 1.0 if weights is None else float(weights[index])
        for node, c in combo.items():
            out[node] = out.get(node, 0.0) + weight * c
    return _clean(out)


def _terms(combo: Combo) -> Terms:
    return tuple(sorted(combo.items(), key=lambda item: repr(item[0])))


def value_of(combo: Combo, values: Dict[Node, float]) -> float:
    return float(sum(c * values[node] for node, c in combo.items()))


class _MulTable:
    def __init__(self, steps: List[IkStep]) -> None:
        self.steps = steps
        self.known: Dict[Tuple[Terms, Terms], Mul] = {}

    @staticmethod
    def _split(combo: Combo) -> Tuple[Terms, float]:
        if len(combo) == 1 and ONE not in combo:
            (node, scale), = combo.items()
            return ((node, 1.0),), float(scale)
        return _terms(combo), 1.0

    def times(self, a: Combo, b: Combo) -> Combo:
        a, b = _clean(a), _clean(b)
        if not a or not b:
            return {}
        if set(a) == {ONE}:
            return _clean({node: c * a[ONE] for node, c in b.items()})
        if set(b) == {ONE}:
            return _clean({node: c * b[ONE] for node, c in a.items()})
        (ta, sa), (tb, sb) = self._split(a), self._split(b)
        key = (ta, tb)
        if key not in self.known:
            self.known[key] = Mul(len(self.known), ta, tb)
            self.steps.append(self.known[key])
        return {self.known[key].node: sa * sb}


def _goal_node(node: Node) -> Node:
    if node == ONE:
        return ONE
    if node[0] == "feat":
        return ("gfeat", node[1], node[2])
    return ("g", node)


def _goal_combo(combo: Combo) -> Combo:
    return {_goal_node(node): c for node, c in combo.items()}


@dataclass
class IkWiring:
    chain: Chain
    so3: Wiring
    design: IkDesign
    caps: np.ndarray                                   # the largest share of every joint, rad
    steps: List[IkStep] = field(default_factory=list)
    turn: List[Node] = field(default_factory=list)
    shares: List[Tuple[Node, Node]] = field(default_factory=list)       # (plus, minus) per joint
    rests: List[List[Node]] = field(default_factory=list)               # after joint 1 .. n
    goal: Optional[List[List[Combo]]] = None                            # the goal matrix (comparison ``matrix``)

    @property
    def muls(self) -> List[Mul]:
        return [s for s in self.steps if isinstance(s, Mul)]

    @property
    def relays(self) -> List[Relay]:
        return [s for s in self.steps if isinstance(s, Relay)]

    @property
    def halves(self) -> List[Half]:
        return [s for s in self.steps if isinstance(s, Half)]

    def goal_feature_values(self, goal_lpy: Sequence[float]) -> Dict[Node, float]:
        lift, pitch, yaw = [float(v) for v in goal_lpy]
        out: Dict[Node, float] = {}
        for c, angle in enumerate((yaw, pitch, lift)):
            out[("gfeat", c, 0)], out[("gfeat", c, 1)] = float(np.cos(angle)), float(np.sin(angle))
        return out

    def evaluate(self, q: Sequence[float], goal_lpy: Sequence[float], so3_values: Optional[Dict[Node, float]] = None,
                 room: Optional[np.ndarray] = None) -> Dict[Node, float]:
        """Exact value of every node.  ``room``: (n, 2) factors on the minus / plus half of each share (joint limits)."""

        values = dict(so3_values) if so3_values is not None else self.so3.evaluate(q)
        tool = self.so3.tool_matrix(values)
        values[NORM] = float(np.hypot(tool[0, 0], tool[1, 0]))
        values.update(self.goal_feature_values(goal_lpy))
        joints = {node: k for k, pair in enumerate(self.shares) for node in pair}
        for step in self.steps:
            if isinstance(step, Mul):
                values[step.node] = value_of(dict(step.a), values) * value_of(dict(step.b), values)
            elif isinstance(step, Relay):
                values[step.node] = float(np.clip(value_of(dict(step.terms), values), -1.0, 1.0))
            else:
                x = value_of(dict(step.terms), values)
                if room is not None and step.node in joints:
                    x -= 1.0 - float(room[joints[step.node], 1 if step.node[2] > 0 else 0])      # the state ring holds this half down
                values[step.node] = float(np.clip(x, 0.0, 1.0))
        return values

    def turn_vector(self, values: Dict[Node, float]) -> np.ndarray:
        """The comparison ``e`` (rad)."""

        return np.array([values[node] for node in self.turn]) / TURN_SCALE

    def share_values(self, values: Dict[Node, float]) -> np.ndarray:
        """Each joint's share, rad."""

        return np.array([self.caps[k] * (values[plus] - values[minus]) for k, (plus, minus) in enumerate(self.shares)])

    def left_vector(self, values: Dict[Node, float]) -> np.ndarray:
        """What the whole arm could not take (rad)."""

        return np.array([values[node] for node in self.rests[-1]]) / TURN_SCALE

    def summary(self) -> Dict[str, int]:
        return {"products": len(self.muls), "sum_populations": len(self.relays), "share_populations": len(self.halves)}


def _goal_steps(wiring: IkWiring) -> List[List[Combo]]:
    """The goal matrix from the three goal rings: the forward chain's own wiring for yaw, pitch, lift."""

    goal = chain_wiring(rules.GOAL_CHAIN, relay=True)
    for step in goal.steps:
        if isinstance(step, Product):
            a = _goal_combo(dict(step.operand))
            wiring.steps.append(Mul(-1, _terms(a), ((_goal_node(step.feature), 1.0),), _goal_node(step.node)))
        elif isinstance(step, Entry):
            wiring.steps.append(Relay(_goal_node(step.node), _terms(_goal_combo(dict(step.terms)))))
    return [[_goal_combo(goal.tool[i][j]) for j in range(3)] for i in range(3)]


def ik_wiring(chain: Chain, so3: Optional[Wiring] = None, design: Optional[IkDesign] = None, ring_size: int = 200) -> IkWiring:
    design = design or IkDesign()
    so3 = so3 or chain_wiring(chain, relay=True)
    wiring = IkWiring(chain, so3, design, rules.share_caps(chain, design, ring_size))
    table = _MulTable(wiring.steps)
    tool = so3.tool

    def gf(c: int, part: int) -> Combo:
        return {("gfeat", c, part): 1.0}

    yaw, pitch, lift = 0, 1, 2
    if design.comparison == "matrix":
        wiring.goal = _goal_steps(wiring)
        components = []
        for i in range(3):
            a, b = (i + 1) % 3, (i + 2) % 3
            terms = []
            for j in range(3):                       # (r_j x g_j)_i = r_aj g_bj - r_bj g_aj
                terms.append(table.times(tool[a][j], wiring.goal[b][j]))
                terms.append(_sum([table.times(tool[b][j], wiring.goal[a][j])], [-1.0]))
            components.append(_sum(terms, [0.5] * len(terms)))
    else:
        e_lift = _sum([table.times(gf(lift, 1), tool[2][2]), table.times(gf(lift, 0), tool[2][1])], [1.0, -1.0])
        e_pitch = _sum([table.times(gf(pitch, 1), {NORM: 1.0}), table.times(gf(pitch, 0), tool[2][0])], [1.0, 1.0])
        e_yaw = _sum([table.times(gf(yaw, 1), tool[0][0]), table.times(gf(yaw, 0), tool[1][0])], [1.0, -1.0])
        cells = []
        for c, combo in ((lift, e_lift), (pitch, e_pitch), (yaw, e_yaw)):
            wiring.steps.append(Relay(("adiff", c), _terms(combo)))
            cells.append({("adiff", c): 1.0})
        d_lift, d_pitch, d_yaw = cells
        components = [
            _sum([table.times(d_lift, tool[0][0]), table.times(d_pitch, tool[1][0])], [1.0, -1.0]),
            _sum([table.times(d_lift, tool[1][0]), table.times(d_pitch, tool[0][0])], [1.0, 1.0]),
            _sum([table.times(d_lift, tool[2][0]), d_yaw], [1.0, 1.0]),
        ]
    for i, combo in enumerate(components):
        wiring.steps.append(Relay(("turn", i), _terms(_sum([combo], [TURN_SCALE]))))
        wiring.turn.append(("turn", i))

    rest: List[Combo] = [{node: 1.0} for node in wiring.turn]
    for k, joint in enumerate(chain.joints):
        axis = [so3.links[k][i][AXES.index(joint.axis)] for i in range(3)]
        asked = _sum([table.times(axis[i], rest[i]) for i in range(3)])                  # TURN_SCALE x the share that is asked for
        scale = 1.0 / (TURN_SCALE * wiring.caps[k])
        plus, minus = ("share", k, 1), ("share", k, -1)
        wiring.steps.append(Half(plus, _terms(_sum([asked], [scale]))))
        wiring.steps.append(Half(minus, _terms(_sum([asked], [-scale]))))
        wiring.shares.append((plus, minus))
        taken = {plus: 1.0, minus: -1.0}
        nodes = []
        for i in range(3):
            if design.cascade:
                combo = _sum([rest[i], table.times(axis[i], taken)], [1.0, -TURN_SCALE * wiring.caps[k]])
            else:
                combo = dict(rest[i])
            node = ("rest", k + 1, i)
            if design.cascade:
                wiring.steps.append(Relay(node, _terms(combo)))
            nodes.append(node)
        if design.cascade:
            rest = [{node: 1.0} for node in nodes]
            wiring.rests.append(nodes)
    if not design.cascade:
        wiring.rests.append(list(wiring.turn))
    return wiring


__all__ = ["Half", "IkStep", "IkWiring", "Mul", "NORM", "Relay", "TURN_SCALE", "ik_wiring", "value_of"]
