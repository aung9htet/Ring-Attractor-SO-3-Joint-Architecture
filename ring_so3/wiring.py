"""How the orientation is computed from the joints' cos and sin, as a list of small steps.

The rate copy (:mod:`ring_so3.rate`) and the NEST network (:mod:`ring_so3.nest.network`)
both follow a :class:`Wiring`, so they compute the same thing in the same order.

Values are named by *nodes*:

* ``("feat", k, 0)`` and ``("feat", k, 1)``: ``cos q_k`` and ``sin q_k`` of joint ``k`` (0-based);
* ``("prod", i)``: a product population, value = (a weighted sum of earlier nodes) x (one feature);
* ``("entry", link, row, col)``: a link-orientation population, value = a weighted sum of
  earlier nodes: one entry of the orientation matrix of that link;
* ``ONE``: the constant 1.

A matrix is a 3 x 3 table of *combos* (``{node: coefficient}``); most combos are a single
node with coefficient +-1.

:func:`chain_wiring` works joint by joint: ``R_k = R_(k-1) · A_k · Rot(axis_k, q_k)``.  The
fixed rotation ``A_k`` only re-labels columns (a weighted sum in general); the joint then
mixes two columns, ``col_u' = col_u·cos + col_v·sin`` and ``col_v' = -col_u·sin + col_v·cos``,
which costs four products per matrix row; the third column is not touched and is not copied.
With ``relay=True`` every new entry that is a sum of several products gets its own
link-orientation population; with ``relay=False`` the sums are passed on as they are.

:func:`flat_wiring` is the all-at-once version of the paper: one node per product feature
that occurs in the end-effector matrix (a product of up to ``n`` factors, built pairwise in
joint order) and one linear readout with the exact matrix ``T``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from .kinematics import AXES, Chain, other_axes, rotation_polynomials

Node = Tuple
Combo = Dict[Node, float]
ONE: Node = ("one",)
EPS = 1e-12


def feature_node(joint: int, part: int) -> Node:
    """``part`` 0: cos, 1: sin."""

    return ("feat", int(joint), int(part))


@dataclass(frozen=True)
class Product:
    index: int
    #: the value that is multiplied: sum of coefficient x node (no constant part)
    operand: Tuple[Tuple[Node, float], ...]
    #: joint whose cos (``part`` 0) or sin (``part`` 1) multiplies it
    joint: int
    part: int

    @property
    def node(self) -> Node:
        return ("prod", self.index)

    @property
    def feature(self) -> Node:
        return feature_node(self.joint, self.part)


@dataclass(frozen=True)
class Entry:
    link: int          # 1 .. n
    row: int
    col: int
    terms: Tuple[Tuple[Node, float], ...]

    @property
    def node(self) -> Node:
        return ("entry", self.link, self.row, self.col)


Step = Union[Product, Entry]
Matrix = List[List[Combo]]


def _add(target: Combo, node: Node, coefficient: float) -> None:
    value = target.get(node, 0.0) + float(coefficient)
    if abs(value) < EPS:
        target.pop(node, None)
    else:
        target[node] = value


def _sum(*combos: Combo) -> Combo:
    out: Combo = {}
    for combo in combos:
        for node, coefficient in combo.items():
            _add(out, node, coefficient)
    return out


def _scaled(combo: Combo, factor: float) -> Combo:
    return {node: coefficient * factor for node, coefficient in combo.items() if abs(coefficient * factor) >= EPS}


def _times_matrix(matrix: Matrix, fixed: np.ndarray) -> Matrix:
    return [[_sum(*[_scaled(matrix[i][m], float(fixed[m][j])) for m in range(3)]) for j in range(3)] for i in range(3)]


def _identity() -> Matrix:
    return [[({ONE: 1.0} if i == j else {}) for j in range(3)] for i in range(3)]


@dataclass
class Wiring:
    chain: Chain
    #: ``"chain"`` (joint by joint) or ``"flat"`` (all product features at once)
    mode: str
    relay: bool
    steps: List[Step] = field(default_factory=list)
    #: orientation of link 1 .. n as tables of combos (empty for ``flat``)
    links: List[Matrix] = field(default_factory=list)
    #: orientation of the end effector
    tool: Matrix = field(default_factory=_identity)

    @property
    def products(self) -> List[Product]:
        return [s for s in self.steps if isinstance(s, Product)]

    @property
    def entries(self) -> List[Entry]:
        return [s for s in self.steps if isinstance(s, Entry)]

    def evaluate(self, q: Sequence[float],
                 feature: Optional[Callable[[int, float], Tuple[float, float]]] = None,
                 product: Optional[Callable[[float, float, Product], float]] = None,
                 entry: Optional[Callable[[float, Entry], float]] = None) -> Dict[Node, float]:
        """Value of every node for joint angles ``q``.  The three callables replace the exact
        ``cos / sin``, product and sum by what a population gives (used by the rate copy)."""

        values: Dict[Node, float] = {ONE: 1.0}
        for k, angle in enumerate(np.asarray(q, dtype=float).reshape(-1)):
            c, s = feature(k, float(angle)) if feature is not None else (float(np.cos(angle)), float(np.sin(angle)))
            values[feature_node(k, 0)], values[feature_node(k, 1)] = c, s
        for step in self.steps:
            if isinstance(step, Product):
                a = sum(coefficient * values[node] for node, coefficient in step.operand)
                b = values[step.feature]
                values[step.node] = product(a, b, step) if product is not None else a * b
            else:
                x = sum(coefficient * values[node] for node, coefficient in step.terms)
                values[step.node] = entry(x, step) if entry is not None else x
        return values

    @staticmethod
    def matrix(values: Dict[Node, float], combos: Matrix) -> np.ndarray:
        return np.array([[sum(c * values[node] for node, c in combos[i][j].items()) for j in range(3)] for i in range(3)])

    def tool_matrix(self, values: Dict[Node, float]) -> np.ndarray:
        return self.matrix(values, self.tool)

    def link_matrices(self, values: Dict[Node, float]) -> List[np.ndarray]:
        return [self.matrix(values, link) for link in self.links]

    def depths(self) -> Dict[Node, int]:
        """Populations a signal passes from a joint ring to each node (a feature is 1)."""

        depth: Dict[Node, int] = {ONE: 0}
        for k in range(self.chain.n):
            depth[feature_node(k, 0)] = depth[feature_node(k, 1)] = 1
        for step in self.steps:
            if isinstance(step, Product):
                depth[step.node] = 1 + max([depth[node] for node, _ in step.operand] + [depth[step.feature]])
            else:
                depth[step.node] = 1 + max(depth[node] for node, _ in step.terms)
        return depth

    def tool_nodes(self) -> List[Node]:
        return sorted({node for row in self.tool for combo in row for node in combo if node != ONE})

    def summary(self) -> Dict[str, int]:
        depth = self.depths()
        return {
            "joints": self.chain.n,
            "products": len(self.products),
            "entries": len(self.entries),
            "depth": max([depth[node] for node in self.tool_nodes()] or [0]),
        }


def _needs_population(combo: Combo) -> bool:
    nodes = [node for node in combo if node != ONE]
    return len(nodes) >= 2 or (len(nodes) == 1 and ONE in combo)


class _ProductTable:
    def __init__(self, steps: List[Step]) -> None:
        self.steps = steps
        self.known: Dict[Tuple, Product] = {}

    def times_feature(self, operand: Combo, joint: int, part: int) -> Combo:
        """Combo of ``operand x feature`` (a constant part of the operand needs no product)."""

        out: Combo = {}
        if ONE in operand:
            _add(out, feature_node(joint, part), operand[ONE])
        rest = {node: c for node, c in operand.items() if node != ONE}
        if not rest:
            return out
        if len(rest) == 1:
            (node, scale), = rest.items()
            key_operand: Tuple[Tuple[Node, float], ...] = ((node, 1.0),)
        else:
            key_operand, scale = tuple(sorted(rest.items())), 1.0
        key = (key_operand, int(joint), int(part))
        if key not in self.known:
            self.known[key] = Product(len(self.known), key_operand, int(joint), int(part))
            self.steps.append(self.known[key])
        _add(out, self.known[key].node, scale)
        return out


def chain_wiring(chain: Chain, relay: bool = True) -> Wiring:
    """Joint by joint.  ``relay``: give every new entry that is a sum its own population."""

    wiring = Wiring(chain, "chain", bool(relay))
    table = _ProductTable(wiring.steps)
    matrix = _identity()
    for k, joint in enumerate(chain.joints):
        moved = _times_matrix(matrix, joint.fixed_matrix())
        a, (u, v) = AXES.index(joint.axis), other_axes(joint.axis)
        new: Matrix = [[{} for _ in range(3)] for _ in range(3)]
        for i in range(3):
            new[i][a] = moved[i][a]
            new[i][u] = _sum(table.times_feature(moved[i][u], k, 0), table.times_feature(moved[i][v], k, 1))
            new[i][v] = _sum(_scaled(table.times_feature(moved[i][u], k, 1), -1.0), table.times_feature(moved[i][v], k, 0))
            if relay:
                for col in (u, v):
                    if _needs_population(new[i][col]):
                        entry = Entry(k + 1, i, col, tuple(sorted(new[i][col].items())))
                        wiring.steps.append(entry)
                        new[i][col] = {entry.node: 1.0}
        matrix = new
        wiring.links.append([[dict(combo) for combo in row] for row in matrix])
    wiring.tool = _times_matrix(matrix, np.array(chain.tool_fixed, dtype=float))
    return wiring


def flat_wiring(chain: Chain) -> Wiring:
    """All product features of the end-effector matrix at once, then the linear readout ``T``."""

    wiring = Wiring(chain, "flat", False)
    table = _ProductTable(wiring.steps)
    nodes: Dict[Tuple[int, ...], Combo] = {}

    def monomial_combo(monomial: Tuple[int, ...]) -> Combo:
        if monomial in nodes:
            return nodes[monomial]
        factors = [(k, part - 1) for k, part in enumerate(monomial) if part]
        if not factors:
            combo: Combo = {ONE: 1.0}
        elif len(factors) == 1:
            combo = {feature_node(*factors[0]): 1.0}
        else:
            last_joint, last_part = factors[-1]
            prefix = list(monomial)
            prefix[last_joint] = 0
            combo = table.times_feature(monomial_combo(tuple(prefix)), last_joint, last_part)
        nodes[monomial] = combo
        return combo

    polys = rotation_polynomials(chain)
    # shorter products first, so that every product's operand exists before it
    for monomial in sorted({m for row in polys for entry in row for m in entry}, key=lambda m: (sum(1 for p in m if p), m)):
        monomial_combo(monomial)
    wiring.tool = [[_sum(*[_scaled(monomial_combo(m), c) for m, c in polys[i][j].items()]) for j in range(3)] for i in range(3)]
    return wiring


def make_wiring(chain: Chain, mode: str = "chain", relay: bool = True) -> Wiring:
    if mode == "chain":
        return chain_wiring(chain, relay)
    if mode == "flat":
        return flat_wiring(chain)
    raise ValueError("mode must be 'chain' or 'flat', got %r" % (mode,))


__all__ = ["Combo", "Entry", "Node", "ONE", "Product", "Wiring", "chain_wiring", "feature_node", "flat_wiring", "make_wiring"]
