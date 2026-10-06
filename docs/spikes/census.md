# The network in spikes: census

Written by `scripts/spike_census.py`. Do not edit by hand.

generic3 with the IK layer, 16,900 neurons, held at one pose for 2 s of network time, 4 threads. "Fire": at least one spike per second. "Spikes per tick": per population, in 50 ms. "Regularity": standard deviation over mean of the gaps between a neuron's spikes (0: a clock; 1: random, like a Poisson process), median over the neurons with six spikes or more.

| Kind of population | Populations | Neurons in each | Fire | Rate of those that fire | Highest rate | Spikes per tick, mean (range) | Regularity |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| joint ring | 3 | 200 | 27 | 38 Hz | 72 Hz | 51 (51 to 52) | 0.02 |
| cos / sin of a joint | 3 | 120 | 71 | 81 Hz | 176 Hz | 287 (278 to 300) | 0.02 |
| product, forward network | 14 | 200 | 110 | 82 Hz | 192 Hz | 447 (374 to 533) | 0.02 |
| sum: entry of a link's orientation | 4 | 60 | 30 | 98 Hz | 190 Hz | 148 (104 to 192) | 0.03 |
| length of a pair (cos of pitch) | 1 | 120 | 58 | 65 Hz | 124 Hz | 187 (187 to 187) | 0.03 |
| output ring | 3 | 100 | 34 | 56 Hz | 148 Hz | 106 (43 to 228) | 0.11 |
| goal ring | 3 | 200 | 27 | 38 Hz | 72 Hz | 51 (51 to 52) | 0.01 |
| cos / sin of a goal angle | 3 | 120 | 67 | 93 Hz | 178 Hz | 312 (309 to 317) | 0.01 |
| product, goal orientation | 14 | 200 | 112 | 84 Hz | 194 Hz | 469 (396 to 566) | 0.02 |
| sum, goal orientation | 4 | 60 | 30 | 99 Hz | 188 Hz | 151 (114 to 193) | 0.03 |
| product, comparison | 18 | 200 | 112 | 78 Hz | 172 Hz | 437 (382 to 482) | 0.02 |
| sum: the comparison e | 3 | 60 | 31 | 87 Hz | 178 Hz | 132 (125 to 145) | 0.06 |
| product, cascade | 10 | 200 | 113 | 79 Hz | 192 Hz | 446 (393 to 534) | 0.03 |
| share half | 6 | 60 | 31 | 172 Hz | 358 Hz | 261 (123 to 343) | 0.05 |
| sum: what is left | 9 | 60 | 32 | 68 Hz | 164 Hz | 110 (93 to 127) | 0.07 |
| gain pair, one side | 6 | 200 | 11 | 86 Hz | 150 Hz | 50 (13 to 84) | 0.23 |
| goal ring of a joint | 3 | 200 | 16 | 155 Hz | 212 Hz | 109 (98 to 114) | 0.09 |

All populations together: 32539 spikes per tick, that is 650784 per second, 38.5 per neuron and second.

The parts that change when nothing is asked of the arm (the turn above: +0.42, +0.32, +0.30 rad; at the goal the comparison is near zero):

| Kind of population | Spikes per tick with a turn asked for, mean (range) | At the goal, mean (range) | Fire, with a turn | Fire, at the goal |
| --- | ---: | ---: | ---: | ---: |
| product, comparison | 437 (382 to 482) | 435 (372 to 485) | 112 | 112 |
| sum: the comparison e | 132 (125 to 145) | 93 (92 to 94) | 31 | 31 |
| product, cascade | 446 (393 to 534) | 402 (375 to 444) | 113 | 111 |
| share half | 261 (123 to 343) | 93 (92 to 96) | 31 | 32 |
| sum: what is left | 110 (93 to 127) | 92 (92 to 93) | 32 | 30 |
| gain pair, one side | 50 (13 to 84) | 54 (52 to 57) | 11 | 12 |
| goal ring of a joint | 109 (98 to 114) | 105 (104 to 107) | 16 | 22 |

Per joint, with the turn asked for: share in units of the largest share the joint takes, spikes per tick of its two halves and of the two sides of its gain pair, and the lead of its goal ring.

| Joint | Share / largest | + half | − half | Gain pair, right | Gain pair, left | Lead (ring indices) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | +1.00 | 330 | 310 | 84 | 13 | +7.8 |
| 2 | -1.00 | 321 | 343 | 13 | 84 | -8.1 |
| 3 | -0.65 | 141 | 123 | 35 | 72 | -5.7 |
