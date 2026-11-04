# Spatial Reusability-Aware Routing

[![ci](https://github.com/Shaurya55555/spatial-reusability-routing/actions/workflows/ci.yml/badge.svg)](https://github.com/Shaurya55555/spatial-reusability-routing/actions/workflows/ci.yml)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)
![License: MIT](https://img.shields.io/badge/license-MIT-green)
![Dependencies: none](https://img.shields.io/badge/dependencies-none-brightgreen)

A C++17 simulator for routing in multi-hop wireless networks that goes beyond the
classic ETX metric. It models two ideas:

1. **Opportunistic (any-path) routing.** A wireless transmission is a broadcast, so
   any node in a *forwarding set* may pick it up. The cost of a hop is averaged over
   all `2^N` reception outcomes (EATX) instead of assuming one fixed next hop.
2. **Spatial reusability.** Two links that are far enough apart can transmit in the
   same time slot, so one of them is effectively free. The simulator finds which
   links can safely be "fused" with a primary transmission and discounts their cost.

It is a clean re-implementation of the algorithms in the LNMIIT mini project report
*Implementation of Spatial Reusability-Aware Routing in Multi-Hop Wireless Networks*
(November 2025, Department of Computer and Communication Engineering), written by
Shaurya Bajpai and Vishisht Trivedi under Dr. Prateek Rathore.

No external dependencies. Plain CMake, a C++17 compiler, and a dependency-free test
suite.

## Results at a glance

On the bundled 9-node mesh (`topologies/mesh9.txt`, source 0, destinations 7 and 8):

| Stage | Expected cost | Reduction vs ETX |
|---|---|---|
| ETX shortest path (Phase 0) | 5.3571 | baseline |
| Opportunistic routing, EATX (Phase 1) | 4.7210 | 11.9% |
| Fusion, sender-only check (Phase 2) | 1.0870 | 77.0% (too optimistic) |
| Fusion, strict two-check (Phase 3) | 1.6667 | 64.7% |
| Fusion, strict + pairwise (extension) | 3.4981 | 25.9% |

The last row is the defensible number. The rows above it show how a looser
interference rule inflates the gain. See [Reading the three fusion rows](#reading-the-three-fusion-rows).

## Quick start

```bash
git clone https://github.com/Shaurya55555/spatial-reusability-routing.git
cd spatial-reusability-routing

cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/spatial_routing topologies/mesh9.txt
```

Sample output:

```
Phase 0  ETX shortest path          cost 5.3571
Phase 1  Opportunistic (EATX)       cost 4.7210  (11.9% lower than ETX)

Phase 2/3  Spatial fusion (primary cost: total)
  sender-only check:     cost 1.0870  (77.0% lower)  primary 4, masked links: 0->1, 1->3, 3->5
  strict two-check:      cost 1.6667  (64.7% lower)  primary 1, masked links: 2->4, 4->5, 5->7
  strict + pairwise:     cost 3.4981  (25.9% lower)  primary 0, masked links: 3->5
```

### CLI

```
spatial_routing <topology> [--primary-cost total|airtime] [--all]
```

| Option | Meaning |
|---|---|
| `--primary-cost total` | A secondary link saves the primary's whole opportunistic cost (the report's wording). Default. |
| `--primary-cost airtime` | A secondary link saves only the primary's expected broadcasts. A tighter reading with smaller gains. |
| `--all` | Print every scenario the tester tried (every node as primary), not just the best one. |

### Topology files

Plain text, one directive per line, `#` starts a comment.

```
# demo
nodes 3
source 0
dest 2
link 0 1 0.5     # undirected link, reception probability 0.5 (ETX = 2)
dlink 1 2 0.8    # directed link 1 -> 2 only
```

`P` is the reception probability, so `ETX = 1/P`. Two nodes interfere when a link
exists between them, in either direction. Several `dest` lines are treated as
any-sink: reaching either one counts.

Included topologies:

| File | Purpose |
|---|---|
| `topologies/linear7.txt` | 7-node chain. Phase 0 baseline, and the case that exposes mutually interfering masked links. |
| `topologies/mesh7.txt` | 7-node mesh where opportunistic routing beats the ETX path. |
| `topologies/mesh9.txt` | 9-node mesh built to contain the hidden terminal problem. |

## How it works

### Phase 0: linear baseline

Path cost is the sum of link ETX values along a Dijkstra shortest path.

### Phase 1: opportunistic cost (EATX)

For a node with forwarding set `F`, the expected cost over all reception outcomes is

```
cost = (1 + sum over non-empty S of  P(S) * min_{v in S} cost[v]) / (1 - P(nobody hears))
```

`S` is the subset of forwarders that actually received the packet, and `P(S)` is its
probability. The best forwarder in `S` carries the packet on. The denominator is the
retry term: the sender rebroadcasts until at least one forwarder hears it. With a
single forwarder this reduces to `ETX + onward cost`, so Phase 1 agrees with the
Phase 0 baseline.

Costs are computed by Bellman-Ford-style rounds until a fixed point is reached. Each
node's forwarding set is the best prefix of its neighbours sorted by cost. Costs
strictly decrease towards the destination, so routes cannot loop.

### Phase 2: spatial fusion

Pick a **primary** node whose transmission pays for airtime. A **secondary** link that
does not interfere with the primary can transmit at the same moment, so its effective
cost drops:

```
attempts' = max(0, attempts - discount)
```

A node is fused only if every link in its forwarding set is a valid secondary link.

### Phase 3: strict interference (hidden terminal fix)

Checking only the secondary's *sender* against the primary is not enough. In
`mesh9.txt`, node 2 is far from primary 6, but node 4 (the receiver of link 2 to 4)
is a neighbour of 6, so 6's signal would drown out 2's at node 4. The strict rule
requires **both** the sender and the receiver to be non-neighbours of the primary.

### Dynamic scenario tester

Every node is tried as the primary. For each, the valid secondary links are found,
fusion is applied, and the source cost is recomputed. The lowest source cost wins.

## Reading the three fusion rows

1. **Sender-only** is the manual Phase 2 rule: only the secondary link's sender is
   checked against the primary. It is too optimistic because of the hidden terminal
   problem. `tests/test_main.cpp` checks the exact `2 -> 4` under primary `6` case.
2. **Strict two-check** is the report's Phase 3 rule: sender and receiver must both
   be non-neighbours of the primary.
3. **Strict + pairwise** is an extension that is not in the report. The report checks
   every secondary link against the primary only. Nothing stops two masked secondary
   links from interfering with each other (on a chain it will happily mask
   `2->3, 3->4, 4->5, 5->6` at once). The pairwise mode also requires the masked
   links to be compatible with each other, using an exhaustive search over which
   nodes may be fused.

## How the report maps to the code

| Report | What it describes | Where |
|---|---|---|
| Phase 0, linear baseline | ETX = 1/P(success); path cost is the sum of link ETX | `linearCost`, `etxDistances` in `src/routing.cpp` |
| Phase 1, mesh and opportunistic logic | Forwarding sets; cost averaged over all `2^N` outcomes | `evalForwardingSet` |
| 6.2 Iterative cost propagation | Bellman-Ford-style rounds instead of Dijkstra | `solve` |
| Phase 2, spatial fusion | Primary pays, secondary rides free; `Effective_ETX = max(0, ETX - cost of P)` | `Fusion`, `solve` |
| Phase 3, strict interference | Sender and receiver both checked | `validSecondary` with `Validation::Strict` |
| 6.3 Dynamic scenario tester | Every node tried as primary, lowest source cost kept | `runScenarioTester` |

## Decisions where the report was ambiguous

- **EATX includes retries.** The report prints the cost as
  `1 + sum(P*Cost) / (1 - P(fail))`. The code puts the whole numerator over the
  denominator, which is the expected cost when the sender rebroadcasts until someone
  hears it.
- **Destination rule.** Destinations have cost 0, so an outcome that reaches one adds
  no onward cost. When only some forwarders receive, the cost is the best receiver's
  cost. An outcome where every forwarder receives takes the best forwarder's cost,
  not zero.
- **Several destinations** are treated as any-sink.
- **Fusion on a forwarding set.** A node gets the discount only if every link in its
  forwarding set is a valid secondary link. For a single link this is exactly
  `Effective_ETX = max(0, ETX - cost of P)`.

## About the numbers

The report's headline result, a drop from 12.4667 to 5.9333 (about 52%) on its 9-node
mesh, came from the authors' original code and link probabilities. The report does
not list those probabilities, so this repo cannot reproduce that exact figure.
`topologies/mesh9.txt` is a different 9-node mesh built to contain the same hidden
terminal situation, and its results are the ones shown above. Do not compare the two
sets of numbers.

## Tests

`tests/test_main.cpp` is a dependency-free suite of hand-calculated checks, run by
`ctest` and by CI on every push:

- linear baseline and the plain sum of link ETX
- EATX against hand-worked `2^N` examples
- Bellman-Ford fixed point and loop freedom
- hidden terminal validation (sender-only allows it, strict rejects it)
- fusion deduction and the `max(0, ...)` floor
- scenario tester invariants (strict never violates itself, loose is never worse)
- pairwise compatibility on a chain and on the mesh
- topology parser, including malformed input

## Limitations

- Interference is modelled as "neighbours in the link graph", not signal-to-noise
  ratios or a physical channel model.
- Reception probabilities are static.
- The `2^N` enumeration caps a forwarding set at 12 nodes, and the pairwise search is
  exhaustive over at most 12 candidate nodes, so it suits small topologies.
- It computes expected routing cost on a graph. It is not a packet-level simulator.

## Project layout

```
include/srr/   network.hpp, routing.hpp
src/           network.cpp (parser), routing.cpp (algorithms), main.cpp (CLI)
tests/         test_main.cpp (hand-calculated checks, no framework)
topologies/    linear7.txt, mesh7.txt, mesh9.txt
.github/       ci.yml (build, test, run all topologies)
```

## Authors

Shaurya Bajpai and Vishisht Trivedi, with guidance from Dr. Prateek Rathore,
LNM Institute of Information Technology.

Licensed under the [MIT License](LICENSE).
