#!/usr/bin/env python3
"""The network in spikes: who fires, how often, and how a value becomes spikes and a number again.

    python3 -B scripts/spike_census.py                                  # writes docs/spikes/ and docs/figures/spikes_*.png
    python3 -B scripts/spike_census.py --chain tiago7 --only census --out /out/spikes_tiago7

Measured in NEST, for part III of TECHNICAL.md:

* **census**: the forward network with the IK layer of one chain, held at a pose, once with a turn
  asked for and once at its goal.  For every kind of population: how many neurons fire, at what
  rates, how many spikes a tick carries, how regular the spike trains are;
* **neuron**: one neuron of a population driven by the spikes of another: its input current, its
  membrane potential and its spikes, beside the tuning curves of its population;
* **forward**: a joint angle as spikes on its ring, as spikes of its cos / sin population, as
  spikes of a product population, and read back as numbers;
* **ik**: a share on its two halves, the two sides of the gain pair, the state ring and the goal
  ring it writes, as the share changes.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ring_so3.chains import get_chain  # noqa: E402
from ring_so3.design import Design  # noqa: E402
from ring_so3.ik.design import IkDesign  # noqa: E402
from ring_so3.ik.pairs import DEVELOPMENT_SEED, pairs  # noqa: E402
from ring_so3.ik.wiring import Half, Mul, Relay  # noqa: E402
from ring_so3.kinematics import lift_pitch_yaw  # noqa: E402
from ring_so3.mapping import RingMap  # noqa: E402

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
INK, GREY, LIGHT = "#1c2330", "#8a94a3", "#d9dee6"
BLUE, GREEN, ORANGE, PURPLE, RED = "#2a78d6", "#1baf7a", "#eb6834", "#7559c7", "#d9443a"
TICK_MS = 50.0


# -- recording ----------------------------------------------------------------------
class Tap:
    """A spike recorder of our own on a group of neurons (the network's recorders are emptied every tick)."""

    def __init__(self, nest, neurons):
        self.nest, self.neurons = nest, neurons
        self.n = len(neurons)
        self.first = int(np.min(nest.GetStatus(neurons, "global_id")))
        self.recorder = nest.Create("spike_recorder")
        nest.Connect(neurons, self.recorder)

    def clear(self):
        self.nest.SetStatus(self.recorder, {"n_events": 0})

    def events(self):
        events = self.nest.GetStatus(self.recorder, "events")[0]
        return np.asarray(events["senders"], dtype=int) - self.first, np.asarray(events["times"], dtype=float)


def train_stats(index, times, n, window_s):
    """Rates and regularity of the spike trains of ``n`` neurons over ``window_s`` seconds."""

    counts = np.bincount(index, minlength=n)[:n].astype(float)
    rates = counts / window_s
    active = rates >= 1.0
    cvs = []
    order = np.argsort(index, kind="stable")
    index_sorted, times_sorted = index[order], times[order]
    starts = np.searchsorted(index_sorted, np.arange(n))
    ends = np.searchsorted(index_sorted, np.arange(n), side="right")
    for i in np.flatnonzero(counts >= 6):
        gaps = np.diff(np.sort(times_sorted[starts[i]:ends[i]]))
        cvs.append(float(np.std(gaps) / np.mean(gaps)))
    return {"neurons": int(n), "active": int(active.sum()), "rate_active_hz": float(rates[active].mean()) if active.any() else 0.0,
            "rate_max_hz": float(rates.max()) if n else 0.0, "spikes_per_tick": float(counts.sum() / (window_s * 1000.0 / TICK_MS)),
            "isi_cv": float(np.median(cvs)) if cvs else None}


def merge(rows):
    """One line for all populations of a kind."""

    cvs = [r["isi_cv"] for r in rows if r["isi_cv"] is not None]
    return {"populations": len(rows), "neurons_each": int(round(np.mean([r["neurons"] for r in rows]))),
            "active_each": float(np.mean([r["active"] for r in rows])),
            "rate_active_hz": float(np.mean([r["rate_active_hz"] for r in rows if r["active"]])) if any(r["active"] for r in rows) else 0.0,
            "rate_max_hz": float(np.max([r["rate_max_hz"] for r in rows])),
            "spikes_per_tick_each": float(np.mean([r["spikes_per_tick"] for r in rows])),
            "spikes_per_tick_low": float(np.min([r["spikes_per_tick"] for r in rows])),
            "spikes_per_tick_high": float(np.max([r["spikes_per_tick"] for r in rows])),
            "isi_cv": float(np.median(cvs)) if cvs else None}


KIND_ORDER = (
    "joint ring", "cos / sin of a joint", "product, forward network", "sum: entry of a link's orientation",
    "length of a pair (cos of pitch)", "output ring",
    "goal ring", "cos / sin of a goal angle", "product, goal orientation", "sum, goal orientation",
    "product, comparison", "sum: the comparison e", "product, cascade", "share half", "sum: what is left",
    "gain pair, one side", "goal ring of a joint",
)


def ik_kinds(network):
    """The kind of every population of the IK layer, from the order of the wiring's steps."""

    kinds, in_cascade = {}, False
    for step in network.ik_wiring.steps:
        node = step.node
        if isinstance(step, Mul):
            kinds[node] = "product, goal orientation" if step.name is not None else ("product, cascade" if in_cascade else "product, comparison")
        elif isinstance(step, Half):
            kinds[node] = "share half"
        elif isinstance(step, Relay):
            if node[0] == "g":
                kinds[node] = "sum, goal orientation"
            elif node[0] in ("turn", "adiff"):
                kinds[node] = "sum: the comparison e"
                in_cascade = in_cascade or node == ("turn", 2)
            else:
                kinds[node] = "sum: what is left"
    return kinds


def census(nest, chain_name, threads, log=print):
    from ring_so3.ik import experiments
    from ring_so3.ik.session import IkSession

    started = time.time()
    chain, network = experiments.build_ik(nest, chain_name, IkDesign(), threads=threads)
    taps = []                                                      # (kind, name, Tap)
    for k, ring in enumerate(network.rings):
        taps.append(("joint ring", "joint ring %d" % (k + 1), Tap(nest, ring.neurons)))
        taps.append(("cos / sin of a joint", "feature %d" % (k + 1), Tap(nest, network.features[k].neurons)))
    for node, population in network.products.items():
        taps.append(("product, forward network", str(node), Tap(nest, population.neurons)))
    for node, population in network.entries.items():
        taps.append(("sum: entry of a link's orientation", str(node), Tap(nest, population.neurons)))
    taps.append(("length of a pair (cos of pitch)", "pitch_norm", Tap(nest, network.pitch_norm.neurons)))
    for name, population in network.outputs.items():
        taps.append(("output ring", name, Tap(nest, population.neurons)))
    for c, ring in enumerate(network.goal_rings):
        taps.append(("goal ring", "goal ring %d" % c, Tap(nest, ring.neurons)))
        taps.append(("cos / sin of a goal angle", "goal feature %d" % c, Tap(nest, network.goal_features[c].neurons)))
    kinds = ik_kinds(network)
    for node, population in network.ik_populations.items():
        taps.append((kinds[node], str(node), Tap(nest, population.neurons)))
    for k, goal in enumerate(network.joint_goal_rings):
        taps.append(("gain pair, one side", "gain left %d" % (k + 1), Tap(nest, goal.share_left)))
        taps.append(("gain pair, one side", "gain right %d" % (k + 1), Tap(nest, goal.share_right)))
        taps.append(("goal ring of a joint", "joint goal ring %d" % (k + 1), Tap(nest, goal.ring.neurons)))

    session = IkSession(network)
    session.follow = False                                         # the arm is held: the turn that is asked for stays
    session.warm_up()
    pair = pairs(chain, 1, DEVELOPMENT_SEED)[0]
    session.place(pair.start)
    circular = np.array([j.circular for j in chain.joints])
    step = pair.goal_pose - pair.start
    step = np.where(circular, np.angle(np.exp(1j * step)), step)
    asked = np.array(lift_pitch_yaw(chain.rotation(pair.start + 0.2 * step)))
    window_s, ticks = 2.0, 40
    out = {"chain": chain_name, "neurons": int(network.neuron_count()), "threads": threads, "window_s": window_s, "conditions": {}}
    for name, goal in (("a turn is asked for", asked), ("at the goal", None)):
        for _ in range(40):                                        # the goal travels on the goal rings and everything settles
            if goal is None:
                session.hold()
            else:
                session.goal = goal
                session.tick()
        for _, _, tap in taps:
            tap.clear()
        last = None
        for _ in range(ticks):
            last = session.hold() if goal is None else session.tick()
        rows = {}
        for kind, label, tap in taps:
            rows.setdefault(kind, []).append(dict(train_stats(*tap.events(), tap.n, window_s), name=label))
        out["conditions"][name] = {
            "kinds": {kind: merge(rows[kind]) for kind in KIND_ORDER if kind in rows},
            "populations": rows, "turn_rad": [float(v) for v in last.turn], "shares_rad": [float(v) for v in last.shares],
            "caps_rad": [float(v) for v in network.ik_wiring.caps], "lead_indices": [float(v) for v in last.lead],
            "total_spikes_per_tick": float(sum(r["spikes_per_tick"] for group in rows.values() for r in group)),
        }
        log("%s, %s: %.0f spikes per tick in all (%.0f s)" % (chain_name, name, out["conditions"][name]["total_spikes_per_tick"], time.time() - started))
    return out


def census_table(result):
    turning, resting = result["conditions"]["a turn is asked for"], result["conditions"]["at the goal"]
    lines = ["# The network in spikes: census", "",
             "Written by `scripts/spike_census.py`. Do not edit by hand.", "",
             "%s with the IK layer, %s neurons, held at one pose for %.0f s of network time, %d threads. "
             "\"Fire\": at least one spike per second. \"Spikes per tick\": per population, in 50 ms. "
             "\"Regularity\": standard deviation over mean of the gaps between a neuron's spikes (0: a clock; 1: random, like a Poisson process), "
             "median over the neurons with six spikes or more." % (
                 result["chain"], format(result["neurons"], ","), result["window_s"], result["threads"]), "",
             "| Kind of population | Populations | Neurons in each | Fire | Rate of those that fire | Highest rate | Spikes per tick, mean (range) | Regularity |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for kind, row in turning["kinds"].items():
        lines.append("| %s | %d | %d | %.0f | %.0f Hz | %.0f Hz | %.0f (%.0f to %.0f) | %s |" % (
            kind, row["populations"], row["neurons_each"], row["active_each"], row["rate_active_hz"], row["rate_max_hz"],
            row["spikes_per_tick_each"], row["spikes_per_tick_low"], row["spikes_per_tick_high"],
            "–" if row["isi_cv"] is None else "%.2f" % row["isi_cv"]))
    lines += ["", "All populations together: %.0f spikes per tick, that is %.0f per second, %.1f per neuron and second." % (
        turning["total_spikes_per_tick"], 20 * turning["total_spikes_per_tick"], 20 * turning["total_spikes_per_tick"] / result["neurons"]), "",
        "The parts that change when nothing is asked of the arm (the turn above: %s rad; at the goal the comparison is near zero):" % (
            ", ".join("%+.2f" % v for v in turning["turn_rad"])), "",
        "| Kind of population | Spikes per tick with a turn asked for, mean (range) | At the goal, mean (range) | Fire, with a turn | Fire, at the goal |",
        "| --- | ---: | ---: | ---: | ---: |"]
    for kind in ("product, comparison", "sum: the comparison e", "product, cascade", "share half", "sum: what is left", "gain pair, one side", "goal ring of a joint"):
        a, b = turning["kinds"][kind], resting["kinds"][kind]
        lines.append("| %s | %.0f (%.0f to %.0f) | %.0f (%.0f to %.0f) | %.0f | %.0f |" % (
            kind, a["spikes_per_tick_each"], a["spikes_per_tick_low"], a["spikes_per_tick_high"],
            b["spikes_per_tick_each"], b["spikes_per_tick_low"], b["spikes_per_tick_high"], a["active_each"], b["active_each"]))
    lines += ["", "Per joint, with the turn asked for: share in units of the largest share the joint takes, spikes per tick of its two halves "
              "and of the two sides of its gain pair, and the lead of its goal ring.", "",
              "| Joint | Share / largest | + half | − half | Gain pair, right | Gain pair, left | Lead (ring indices) |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    halves = {r["name"]: r for r in turning["populations"]["share half"]}
    gains = {r["name"]: r for r in turning["populations"]["gain pair, one side"]}
    for k, (share, cap) in enumerate(zip(turning["shares_rad"], turning["caps_rad"])):
        lines.append("| %d | %+.2f | %.0f | %.0f | %.0f | %.0f | %+.1f |" % (
            k + 1, share / cap, halves[str(("share", k, 1))]["spikes_per_tick"], halves[str(("share", k, -1))]["spikes_per_tick"],
            gains["gain right %d" % (k + 1)]["spikes_per_tick"], gains["gain left %d" % (k + 1)]["spikes_per_tick"], turning["lead_indices"][k]))
    return lines


# -- one neuron ---------------------------------------------------------------------------
def neuron_run(nest, threads):
    """A scalar population driven by the spikes of another whose value steps from 0 to 0.7; one neuron watched closely."""

    from ring_so3.nest import calibrate
    from ring_so3.nest.populations import connect_weights, create_population, decoded_weights, set_value

    design = Design()
    cal = calibrate.load(design)
    tuning = design.entry
    calibrate._kernel(nest, threads, 3)
    source = create_population(nest, tuning, "source", seed=1)
    target = create_population(nest, tuning, "target", seed=2)
    connect_weights(nest, source.neurons, target.neurons, decoded_weights(cal.decoders["source"], np.array([[1.0]]), target))
    cut = (tuning.model.rheobase - tuning.bias) / tuning.gain                      # the value at which each neuron starts to fire
    candidates = np.flatnonzero((tuning.encoders[:, 0] > 0) & (cut > 0.25) & (cut < 0.4))
    watched = int(candidates[0])
    meter = nest.Create("multimeter", params={"interval": 0.1, "record_from": ["V_m", "I_syn_ex", "I_syn_in"]})
    nest.Connect(meter, target.neurons[watched])
    source_tap, target_tap = Tap(nest, source.neurons), Tap(nest, target.neurons)
    steps = ((0.0, 250.0), (0.7, 350.0))                                           # value of the source, for how long (ms)
    for value, duration in steps:
        set_value(nest, source, [value])
        nest.Simulate(duration)
    events = nest.GetStatus(meter, "events")[0]
    index, times = target_tap.events()
    source_index, source_times = source_tap.events()
    x = np.linspace(-1.0, 1.0, 201)
    return {
        "watched": watched, "encoder": float(tuning.encoders[watched, 0]), "intercept": float(cut[watched]),
        "bias_pa": float(tuning.bias[watched]), "gain_pa": float(tuning.gain[watched]), "rheobase_pa": float(tuning.model.rheobase),
        "threshold_mv": float(tuning.model.V_th), "steps": steps,
        "t_ms": np.asarray(events["times"], dtype=float), "v_m": np.asarray(events["V_m"], dtype=float),
        "i_ex": np.asarray(events["I_syn_ex"], dtype=float), "i_in": np.asarray(events["I_syn_in"], dtype=float),
        "spike_times_ms": np.sort(times[index == watched]), "source_index": source_index, "source_times": source_times,
        "source_order": np.argsort(np.where(tuning.encoders[:, 0] > 0, cut, 10.0 - cut)),          # + neurons by intercept, then - neurons
        "x": x, "curves": tuning.rates(x[:, None]), "encoders": tuning.encoders[:, 0].copy(), "cut": cut,
    }


def figure_neuron(plt, data, path):
    fig = plt.figure(figsize=(11.0, 6.4))
    grid = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.35], hspace=0.32, wspace=0.2)
    ax = fig.add_subplot(grid[:, 0])
    for i in range(data["curves"].shape[1]):
        ax.plot(data["x"], data["curves"][:, i], color=BLUE if data["encoders"][i] > 0 else ORANGE, lw=0.8, alpha=0.55)
    ax.plot(data["x"], data["curves"][:, data["watched"]], color=INK, lw=2.4)
    ax.plot([], [], color=BLUE, lw=1.5, label="fire more for larger values (encoder +1)")
    ax.plot([], [], color=ORANGE, lw=1.5, label="fire more for smaller values (encoder −1)")
    ax.plot([], [], color=INK, lw=2.4, label="the neuron watched on the right")
    ax.set_xlabel("value the population holds")
    ax.set_ylabel("firing rate for a constant input (Hz)")
    ax.set_title("60 neurons, 60 different ramps")
    ax.legend(fontsize=8, loc="upper center")
    ax.set_ylim(0, 260)

    t0 = data["steps"][0][1]
    lo, hi = t0 - 150.0, t0 + 300.0
    top = fig.add_subplot(grid[0, 1])
    rank = np.empty_like(data["source_order"])
    rank[data["source_order"]] = np.arange(len(data["source_order"]))
    keep = (data["source_times"] >= lo) & (data["source_times"] <= hi)
    top.scatter(data["source_times"][keep] - t0, rank[data["source_index"][keep]], s=3, color=GREY, linewidths=0)
    top.set_ylabel("sending neuron\n(sorted)")
    top.set_title("Spikes in: the sending population steps from the value 0 to 0.7")
    top.tick_params(labelbottom=False)
    mid = fig.add_subplot(grid[1, 1], sharex=top)
    keep = (data["t_ms"] >= lo) & (data["t_ms"] <= hi)
    total = data["bias_pa"] + data["i_ex"] + data["i_in"]
    mid.plot(data["t_ms"][keep] - t0, total[keep], color=PURPLE, lw=1.0)
    mid.axhline(data["rheobase_pa"], color=GREY, lw=1, ls="--")
    mid.text(lo - t0 + 4, data["rheobase_pa"] + 12, "rheobase %.1f pA: above it the neuron fires" % data["rheobase_pa"], fontsize=8, color=GREY)
    mid.set_ylabel("input current\n(pA)")
    mid.set_title("Current: its bias plus one small alpha-shaped pulse per incoming spike")
    mid.tick_params(labelbottom=False)
    bottom = fig.add_subplot(grid[2, 1], sharex=top)
    bottom.plot(data["t_ms"][keep] - t0, data["v_m"][keep], color=INK, lw=1.0)
    bottom.axhline(data["threshold_mv"], color=GREY, lw=1, ls="--")
    for spike in data["spike_times_ms"]:
        if lo <= spike <= hi:
            bottom.plot([spike - t0, spike - t0], [data["threshold_mv"], data["threshold_mv"] + 5.0], color=RED, lw=1.4)
    bottom.set_ylim(min(-2.0, float(np.min(data["v_m"][keep])) - 1.0), data["threshold_mv"] + 7)
    bottom.set_ylabel("membrane\npotential (mV)")
    bottom.set_xlabel("time since the step (ms)")
    bottom.set_title("Spikes out: at the threshold of %.0f mV the neuron fires (red) and starts again from 0" % data["threshold_mv"])
    top.set_xlim(lo - t0, hi - t0)
    fig.savefig(path)
    plt.close(fig)


# -- a joint angle through the forward network ------------------------------------------
def decode(index, times, decoders, t_ms, tau_ms=20.0):
    """Decoded values over time: every spike adds ``decoder / tau`` and the sum decays with ``tau`` (a causal low-pass)."""

    out = np.zeros((t_ms.size, decoders.shape[1]))
    order = np.argsort(times)
    index, times = index[order], times[order]
    state, position = np.zeros(decoders.shape[1]), 0
    previous = t_ms[0]
    for i, now in enumerate(t_ms):
        state *= np.exp(-(now - previous) / tau_ms)
        while position < times.size and times[position] <= now:
            state += decoders[index[position]] * (1000.0 / tau_ms) * np.exp(-(now - times[position]) / tau_ms)
            position += 1
        out[i] = state
        previous = now
    return out


def forward_run(nest, threads):
    """generic2 (R = Rx(q1) Ry(q2)): joint 1 turns, joint 2 is held; ring 1, its cos / sin population and the product sin q1 sin q2."""

    from ring_so3.nest import calibrate
    from ring_so3.nest.network import FkNetwork, NetworkOptions

    design = Design()
    cal = calibrate.load(design)
    chain = get_chain("generic2")
    network = FkNetwork(nest, chain, cal, design, NetworkOptions(threads=threads))
    product = next(step for step in network.wiring.products
                   if step.operand == ((("feat", 0, 1), 1.0),) and (step.joint, step.part) == (1, 1))          # sin q1 times sin q2
    taps = {"ring": Tap(nest, network.rings[0].neurons), "feature": Tap(nest, network.features[0].neurons),
            "product": Tap(nest, network.products[product.node].neurons)}
    q2 = 0.9

    def q1_at(t):
        return 0.5 + 1.7 * min(max((t - 1.0) / 1.0, 0.0), 1.0)
    for _ in range(20):
        network.step([q1_at(0.0), q2])
    for tap in taps.values():
        tap.clear()
    for _ in range(10):                                             # half a second that is decoded but not drawn
        network.step([q1_at(0.0), q2])
    start_ms = network.t_ms
    ticks, written, cells = [], [], []
    for i in range(60):
        t = (i + 1) * TICK_MS / 1000.0
        tick = network.step([q1_at(t), q2])
        ticks.append(t)
        written.append(network.rings[0].target_index)
        cells.append(float(tick.tool[1, 0]))                        # the entry sin q1 sin q2 as the readout cells hold it
    t_ms = np.arange(-500.0, 3000.0 + 1e-9, 2.0)
    events = {name: tap.events() for name, tap in taps.items()}
    events = {name: (index, times - start_ms) for name, (index, times) in events.items()}
    tuning = network.features[0].tuning
    feature_order = np.argsort(np.arctan2(tuning.encoders[:, 1], tuning.encoders[:, 0]))
    product_tuning = design.product
    group = np.arange(product_tuning.n) % 4
    product_cut = (product_tuning.model.rheobase - product_tuning.bias) / product_tuning.gain
    product_order = np.lexsort((product_cut, group))
    q1 = np.array([q1_at(t / 1000.0) for t in t_ms])
    return {
        "events": events, "t_ms": t_ms, "q1": q1, "q2": q2, "ticks_s": np.array(ticks), "written": np.array(written), "cells": np.array(cells),
        "feature_order": feature_order, "product_order": product_order,
        "feature_decoded": decode(*events["feature"], cal.decoders["feature"], t_ms),
        "product_decoded": decode(*events["product"], cal.decoders["product"], t_ms)[:, 0],
        "ring_size": network.rings[0].size,
    }


def raster(ax, index, times, order=None, color=INK, size=2.0):
    if order is not None:
        rank = np.empty_like(order)
        rank[order] = np.arange(order.size)
        index = rank[index]
    ax.scatter(times / 1000.0, index, s=size, color=color, linewidths=0)


def figure_forward(plt, data, path):
    fig, axes = plt.subplots(4, 1, figsize=(10.5, 9.4), sharex=True, gridspec_kw={"height_ratios": [1.15, 1.0, 1.25, 1.2], "hspace": 0.28})
    t = data["t_ms"] / 1000.0
    raster(axes[0], *data["events"]["ring"], color=BLUE)
    axes[0].plot(data["ticks_s"], data["written"], color=ORANGE, lw=1.2, label="where the encoder writes the angle")
    axes[0].set_ylabel("ring neuron")
    axes[0].set_title("1. Joint ring 1: a bump of about 27 firing neurons at the joint angle, which turns from 29° to 126°")
    axes[0].legend(fontsize=8, loc="upper left")
    low, high = np.min(data["written"]) - 30, np.max(data["written"]) + 30
    axes[0].set_ylim(low, high)
    raster(axes[1], *data["events"]["feature"], order=data["feature_order"], color=PURPLE)
    axes[1].set_ylabel("neuron, by the\ndirection it prefers")
    axes[1].set_title("2. Its cos / sin population: every ring spike drives the neurons whose direction lies near the angle")
    raster(axes[2], *data["events"]["product"], order=data["product_order"], color=GREEN)
    for edge in (50, 100, 150):
        axes[2].axhline(edge - 0.5, color=LIGHT, lw=1)
    axes[2].set_yticks([25, 75, 125, 175])
    axes[2].set_yticklabels(["a + b", "a − b", "−a + b", "−a − b"])
    axes[2].set_ylabel("neuron, by what\nit responds to")
    axes[2].set_title("3. A product population holds the pair a = sin q1, b = sin q2: four groups, each sees one sum or difference")
    axes[3].plot(t, np.cos(data["q1"]), color=GREY, lw=1, ls="--")
    axes[3].plot(t, np.sin(data["q1"]), color=GREY, lw=1, ls="--")
    axes[3].plot(t, np.sin(data["q1"]) * np.sin(data["q2"]), color=GREY, lw=1, ls="--", label="exact")
    axes[3].plot(t, data["feature_decoded"][:, 0], color=PURPLE, lw=1.3, label="cos q1 from the spikes of 2")
    axes[3].plot(t, data["feature_decoded"][:, 1], color=BLUE, lw=1.3, label="sin q1 from the spikes of 2")
    axes[3].plot(t, data["product_decoded"], color=GREEN, lw=1.3, label="sin q1 · sin q2 from the spikes of 3")
    axes[3].plot(data["ticks_s"], data["cells"], "o", ms=2.6, color=INK, label="the same entry in a readout cell, once per tick")
    axes[3].set_ylabel("value")
    axes[3].set_xlabel("time (s)")
    axes[3].set_title("4. Read back as numbers: each spike adds its neuron's weight, and the sum fades with 20 ms")
    axes[3].legend(fontsize=7.5, ncol=3, loc="lower left")
    axes[3].set_ylim(-1.15, 1.25)
    axes[3].set_xlim(0, 3.0)
    fig.savefig(path)
    plt.close(fig)


# -- a share through the gain pair onto a goal ring ---------------------------------------
def ik_run(nest, threads):
    from ring_so3.nest import calibrate, ik_calibrate
    from ring_so3.nest.goal_ring import WrittenRingSettings, build_written_ring
    from ring_so3.nest.populations import connect_weights, create_population, decoded_weights, set_value
    from ring_so3.nest.ring import build_joint_ring

    design = Design()
    base, ik = calibrate.load(design), ik_calibrate.load(design)
    calibrate._kernel(nest, threads, 5)
    mapping = RingMap(int(base.settings["ring_size"]), -np.pi, np.pi, True)
    state = build_joint_ring(nest, "state", mapping, mode="fractional")
    one = np.array([[1.0]])
    sources, halves = [], []
    for h in range(2):
        source = create_population(nest, design.entry, "source%d" % h, seed=300 + h)
        half = create_population(nest, design.entry, "half%d" % h, seed=500 + h)
        connect_weights(nest, source.neurons, half.neurons, decoded_weights(base.decoders["source"], one, half))
        sources.append(source)
        halves.append((half, ik.decoders["half"]))
    settings = WrittenRingSettings()
    goal = build_written_ring(nest, "goal", mapping, state, halves[0], halves[1], settings)
    taps = {"plus": Tap(nest, halves[0][0].neurons), "minus": Tap(nest, halves[1][0].neurons), "right": Tap(nest, goal.share_right),
            "left": Tap(nest, goal.share_left), "state": Tap(nest, state.neurons), "goal": Tap(nest, goal.ring.neurons)}
    position = 100.37
    angle = mapping.angle(position)
    segments = ((0.0, 1.0), (0.3, 1.0), (1.0, 1.0), (0.0, 0.75), (-0.6, 1.25))                    # share (in largest shares), seconds
    for source in sources:
        set_value(nest, source, [0.0])
    for _ in range(20):
        state.drive(nest, angle)
        nest.Simulate(TICK_MS)
        state.read(nest)
        goal.ring.read(nest)
    for tap in taps.values():
        tap.clear()
    start_ms = float(nest.GetKernelStatus("biological_time"))
    ticks, lead, shares, history = [], [], [], []
    elapsed = 0.0
    for share, seconds in segments:
        set_value(nest, sources[0], [max(share, 0.0)])
        set_value(nest, sources[1], [max(-share, 0.0)])
        for _ in range(int(round(seconds * 1000.0 / TICK_MS))):
            state.drive(nest, angle)
            nest.Simulate(TICK_MS)
            state.read(nest)
            history = (history + [goal.ring.read(nest)])[-3:]
            elapsed += TICK_MS / 1000.0
            index = mapping.population_vector_index(np.sum(history, axis=0))
            ticks.append(elapsed)
            lead.append(mapping.index_error(index, state.index) if np.isfinite(index) and np.isfinite(state.index) else np.nan)
            shares.append(share)
    events = {name: tap.events() for name, tap in taps.items()}
    events = {name: (index, times - start_ms) for name, (index, times) in events.items()}
    tuning = design.entry
    cut = (tuning.model.rheobase - tuning.bias) / tuning.gain
    return {"events": events, "ticks_s": np.array(ticks), "lead": np.array(lead), "shares": np.array(shares), "shift": int(settings.shift),
            "position": position, "half_order": np.argsort(np.where(tuning.encoders[:, 0] > 0, cut, 10.0 - cut)), "segments": segments,
            "settings": settings.describe()}


def figure_ik(plt, data, path):
    fig, axes = plt.subplots(4, 1, figsize=(10.5, 9.6), sharex=True, gridspec_kw={"height_ratios": [1.0, 1.15, 1.25, 0.9], "hspace": 0.3})
    n = data["half_order"].size
    raster(axes[0], *data["events"]["plus"], order=data["half_order"], color=ORANGE)
    index, times = data["events"]["minus"]
    rank = np.empty_like(data["half_order"])
    rank[data["half_order"]] = np.arange(n)
    axes[0].scatter(times / 1000.0, rank[index] + n + 6, s=2.0, color=PURPLE, linewidths=0)
    axes[0].set_yticks([n / 2, n + 6 + n / 2])
    axes[0].set_yticklabels(["+ half", "− half"])
    axes[0].set_title("1. The share on its two halves, 60 neurons each: which neurons of a half fire, and how fast, says how large it is")
    centre = data["position"]
    for name, color, offset in (("right", ORANGE, 0), ("left", PURPLE, 44)):
        index, times = data["events"][name]
        keep = np.abs(index - centre) <= 20
        axes[1].scatter(times[keep] / 1000.0, index[keep] - centre + 20 + offset, s=2.0, color=color, linewidths=0)
    axes[1].set_yticks([20, 64])
    axes[1].set_yticklabels(["right side", "left side"])
    axes[1].set_title("2. The gain pair: both sides fire under the state bump at rest; the share raises one side and lowers the other")
    axes[2].axhspan(centre - 5.5, centre + 5.5, color=BLUE, alpha=0.16, lw=0, label="state ring: the 11 neurons at the centre of its bump (where the joint is)")
    index, times = data["events"]["goal"]
    keep = np.abs(index - centre) <= 26
    axes[2].scatter(times[keep] / 1000.0, index[keep], s=2.0, color=PURPLE, linewidths=0, label="goal ring: spikes")
    axes[2].set_ylabel("ring neuron")
    axes[2].set_ylim(centre - 22, centre + 27)
    axes[2].set_title("3. The goal ring: the right side writes %d neurons up the ring, the left side %d down; its activity leans the share's way" % (data["shift"], data["shift"]))
    axes[2].legend(fontsize=8, loc="upper left", markerscale=4, ncol=2)
    axes[3].plot(data["ticks_s"], data["shift"] * data["shares"], color=GREY, lw=1.2, ls="--", label="share × %d indices" % data["shift"])
    axes[3].plot(data["ticks_s"], data["lead"], color=INK, lw=1.6, label="lead of the goal ring over the state ring")
    axes[3].axhline(0, color=LIGHT, lw=1)
    axes[3].set_ylabel("ring indices")
    axes[3].set_xlabel("time (s)")
    axes[3].set_title("4. Read as a lead: the centre of the goal ring's spikes minus the centre of the state ring's")
    axes[3].legend(fontsize=8, loc="upper left")
    edge = 0.0
    for share, seconds in data["segments"]:
        for axis in axes:
            axis.axvline(edge, color=LIGHT, lw=1, zorder=0)
        axes[0].text(edge + 0.03, 2 * n + 9, "share %+.1f" % share, fontsize=8, color=GREY, va="bottom")
        edge += seconds
    axes[0].set_ylim(-3, 2 * n + 22)
    axes[3].set_xlim(0, edge)
    fig.savefig(path)
    plt.close(fig)


def jsonable(value):
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, float):
        return round(value, 5) if np.isfinite(value) else None
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--chain", default="generic3", help="chain of the census")
    parser.add_argument("--only", choices=("census", "neuron", "forward", "ik"), action="append", default=None)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "spikes"), help="folder for census.json and census.md")
    parser.add_argument("--figures", default=os.path.join(ROOT, "docs", "figures"), help="folder for spikes_*.png")
    args = parser.parse_args(argv)
    parts = args.only or ["census", "neuron", "forward", "ik"]

    import nest
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 9.5, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": GREY,
                         "axes.labelcolor": INK, "xtick.color": GREY, "ytick.color": GREY, "text.color": INK,
                         "legend.frameon": False, "figure.dpi": 150, "savefig.bbox": "tight", "axes.titlesize": 9.5,
                         "axes.titleweight": "bold", "axes.titlelocation": "left"})
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.figures, exist_ok=True)
    notes_path = os.path.join(args.out, "figures.json")             # the numbers behind the three figures
    notes = {}
    if os.path.exists(notes_path):
        with open(notes_path, encoding="utf-8") as stream:
            notes = json.load(stream)
    if "census" in parts:
        result = census(nest, args.chain, args.threads)
        with open(os.path.join(args.out, "census.json"), "w", encoding="utf-8") as stream:
            json.dump(jsonable(result), stream, separators=(",", ":"))
        with open(os.path.join(args.out, "census.md"), "w", encoding="utf-8") as stream:
            stream.write("\n".join(census_table(result)) + "\n")
        print("written:", os.path.relpath(os.path.join(args.out, "census.md"), ROOT))
    if "neuron" in parts:
        data = neuron_run(nest, args.threads)
        figure_neuron(plt, data, os.path.join(args.figures, "spikes_neuron.png"))
        spikes = data["spike_times_ms"]
        after = spikes[spikes > data["steps"][0][1] + 60.0]
        print("written: spikes_neuron.png | watched neuron: starts to fire at the value %.2f, bias %.0f pA, gain %.0f pA; "
              "%d spikes before the step, rate after it %.0f Hz (constant-input formula at 0.7: %.0f Hz)" % (
                  data["intercept"], data["bias_pa"], data["gain_pa"], int(np.sum(spikes < data["steps"][0][1])),
                  1000.0 * (after.size - 1) / (after[-1] - after[0]) if after.size > 1 else 0.0,
                  float(data["curves"][np.argmin(np.abs(data["x"] - 0.7)), data["watched"]])))
        keep = data["t_ms"] > data["steps"][0][1] + 60.0
        total = data["bias_pa"] + data["i_ex"] + data["i_in"]
        notes["neuron"] = {"starts_to_fire_at": data["intercept"], "bias_pa": data["bias_pa"], "gain_pa": data["gain_pa"],
                           "rheobase_pa": data["rheobase_pa"], "spikes_before_step": int(np.sum(spikes < data["steps"][0][1])),
                           "rate_after_step_hz": 1000.0 * (after.size - 1) / (after[-1] - after[0]) if after.size > 1 else 0.0,
                           "rate_formula_hz": float(data["curves"][np.argmin(np.abs(data["x"] - 0.7)), data["watched"]]),
                           "current_before_pa": float(np.mean(total[data["t_ms"] < data["steps"][0][1]])),
                           "current_after_pa": float(np.mean(total[keep])), "current_after_sd_pa": float(np.std(total[keep])),
                           "first_spike_after_step_ms": float(spikes[spikes >= data["steps"][0][1]][0] - data["steps"][0][1]) if spikes.size else None,
                           "incoming_spikes_per_s_before": float(np.sum(data["source_times"] < data["steps"][0][1]) / (data["steps"][0][1] / 1000.0)),
                           "incoming_spikes_per_s_after": float(np.sum(data["source_times"] >= data["steps"][0][1]) / (data["steps"][1][1] / 1000.0))}
    if "forward" in parts:
        data = forward_run(nest, args.threads)
        figure_forward(plt, data, os.path.join(args.figures, "spikes_forward.png"))
        late = data["t_ms"] > 2400.0
        print("written: spikes_forward.png | after the move: cos q1 %.3f (exact %.3f), sin q1 %.3f (%.3f), product %.3f (%.3f)" % (
            data["feature_decoded"][late, 0].mean(), np.cos(data["q1"][-1]), data["feature_decoded"][late, 1].mean(), np.sin(data["q1"][-1]),
            data["product_decoded"][late].mean(), np.sin(data["q1"][-1]) * np.sin(data["q2"])))
        # how late each stage is: the shift that best lines the decoded curve up with the exact one while the joint turns
        moving = (data["t_ms"] > 900.0) & (data["t_ms"] < 2300.0)
        step_ms = float(data["t_ms"][1] - data["t_ms"][0])

        def lag(decoded, exact):
            best = min(range(0, 200), key=lambda k: float(np.mean((decoded[moving] - np.roll(exact, k)[moving]) ** 2)))
            return best * step_ms / 1000.0
        ring_index, ring_times = data["events"]["ring"]
        early = (data["t_ms"] > 0.0) & (data["t_ms"] < 900.0)
        notes["forward"] = {"cos_after_move": float(data["feature_decoded"][late, 0].mean()), "cos_exact": float(np.cos(data["q1"][-1])),
                            "sin_after_move": float(data["feature_decoded"][late, 1].mean()), "sin_exact": float(np.sin(data["q1"][-1])),
                            "product_after_move": float(data["product_decoded"][late].mean()),
                            "product_exact": float(np.sin(data["q1"][-1]) * np.sin(data["q2"])),
                            "lag_cos_sin_s": lag(data["feature_decoded"][:, 1], np.sin(data["q1"])),
                            "lag_product_s": lag(data["product_decoded"], np.sin(data["q1"]) * np.sin(data["q2"])),
                            "ripple_sin_sd": float(np.std(data["feature_decoded"][early, 1])),
                            "ripple_product_sd": float(np.std(data["product_decoded"][early])),
                            "ring_spikes_per_s": float(np.sum((ring_times > 0) & (ring_times < 900.0)) / 0.9)}
    if "ik" in parts:
        data = ik_run(nest, args.threads)
        figure_ik(plt, data, os.path.join(args.figures, "spikes_ik.png"))
        edges = np.cumsum([0.0] + [seconds for _, seconds in data["segments"]])
        summary = []
        for (share, _), a, b in zip(data["segments"], edges[:-1], edges[1:]):
            keep = (data["ticks_s"] > a + 0.4) & (data["ticks_s"] <= b)
            summary.append("share %+.1f: lead %+.1f" % (share, np.nanmean(data["lead"][keep])))
        print("written: spikes_ik.png |", "; ".join(summary))
        steps = []
        for i in range(1, len(data["segments"])):
            before = (data["ticks_s"] > edges[i] - 0.4) & (data["ticks_s"] <= edges[i])
            after = (data["ticks_s"] > edges[i + 1] - 0.4) & (data["ticks_s"] <= edges[i + 1])
            old, new = float(np.nanmean(data["lead"][before])), float(np.nanmean(data["lead"][after]))
            later = np.flatnonzero((data["ticks_s"] > edges[i]) & ((data["lead"] - old) * np.sign(new - old) >= 0.63 * abs(new - old)))
            steps.append({"from_share": data["segments"][i - 1][0], "to_share": data["segments"][i][0], "lead_before": old, "lead_after": new,
                          "time_to_63_percent_s": float(data["ticks_s"][later[0]] - edges[i]) if later.size else None})
        notes["ik"] = {"lead_by_segment": [{"share": share, "lead": float(np.nanmean(data["lead"][(data["ticks_s"] > a + 0.4) & (data["ticks_s"] <= b)]))}
                                           for (share, _), a, b in zip(data["segments"], edges[:-1], edges[1:])],
                       "steps": steps, "settings": data["settings"]}
    with open(notes_path, "w", encoding="utf-8") as stream:
        json.dump(jsonable(notes), stream, indent=1, sort_keys=True)
    print("written:", os.path.relpath(notes_path, ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
