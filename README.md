# Spatial Reusability-Aware Routing

A C++17 simulator for routing in multi-hop wireless networks that goes beyond
the ETX metric. It models the broadcast nature of wireless links (opportunistic,
any-path routing) and exploits **spatial reusability**: two links that are far
enough apart can transmit in the same time slot, so one of them is effectively
free.

This is a clean re-implementation of the algorithms in the LNMIIT mini project
report *Implementation of Spatial Reusability-Aware Routing in Multi-Hop Wireless
Networks* (November 2025, Department of Computer and Communication Engineering,
guided by Dr. Prateek Rathore), written by Shaurya Bajpai and Vishisht Trivedi.
The report describes the method in four phases. Each phase maps to code here.

## How the report maps to the code

| Report | What it describes | Where |
|---|---|---|
| Phase 0, linear baseline | ETX = 1/P(success); path cost is the sum of link ETX | `linearCost`, `etxDistances` (`src/routing.cpp`), `topologies/linear7.txt` |
| Phase 1, mesh and opportunistic logic | Forwarding sets; cost averaged over all 2^N reception outcomes (EATX) | `evalForwardingSet`, `topologies/mesh7.txt` |
| 6.2 Iterative cost propagation | Bellman-Ford-style rounds instead of Dijkstra | `solve` |
| Phase 2, spatial fusion | Primary (pays) and Secondary (free rider); `Effective_ETX = max(0, ETX - cost of P)` | `Fusion`, `solve` |
| Phase 3, strict interference | Hidden terminal problem fixed by checking both sender and receiver | `validSecondary` with `Validation::Strict`, `topologies/mesh9.txt` |
| 6.3 Dynamic scenario tester | Every node tried as Primary, valid links found, lowest source cost kept | `runScenarioTester` |

## Build and run

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/spatial_routing topologies/mesh9.txt --all
./build/spatial_routing topologies/mesh9.txt --primary-cost airtime
```

Topology files are plain text (`nodes`, `source`, `dest`, `link U V P`,
`dlink U V P`, `#` comments). `P` is the reception probability, so ETX = 1/P.
Two nodes interfere when a link exists between them.

Sample run on the 9-node mesh (source 0, destinations 7 and 8):

```
Phase 0  ETX shortest path          cost 5.3571
Phase 1  Opportunistic (EATX)       cost 4.7210  (11.9% lower than ETX)

Phase 2/3  Spatial fusion (primary cost: total)
  sender-only check:     cost 1.0870  (77.0% lower)  primary 4, masked links: 0->1, 1->3, 3->5
  strict two-check:      cost 1.6667  (64.7% lower)  primary 1, masked links: 2->4, 4->5, 5->7
  strict + pairwise:     cost 3.4981  (25.9% lower)  primary 0, masked links: 3->5
```

## Reading the three fusion rows

1. **Sender-only** is the manual Phase 2 rule: only the secondary link's sender is
   checked against the primary. It is too optimistic because of the hidden
   terminal problem. In `mesh9.txt`, node 2 is far from node 6, but node 4 (the
   receiver of link 2 to 4) is a neighbour of 6, so 6's signal would drown out 2's
   at node 4. `tests/test_main.cpp` checks this exact case.
2. **Strict two-check** is the report's Phase 3 rule: sender and receiver must both
   be non-neighbours of the primary.
3. **Strict + pairwise** is an extension that is not in the report. The report
   checks every secondary link against the primary only. Nothing stops two masked
   secondary links from interfering with each other (on a chain it will happily
   mask `2->3, 3->4, 4->5, 5->6` at once). The pairwise mode also requires the
   masked links to be compatible with each other, using an exhaustive search over
   which nodes may be fused. It gives the defensible number.

## Decisions where the report was ambiguous

- **EATX includes retries.** The report prints the cost as `1 + sum(P*Cost)/(1 - P(fail))`.
  The code uses `(1 + sum over non-empty S of P(S) * min cost in S) / (1 - P(fail))`,
  the expected cost when the sender rebroadcasts until someone hears it. With a
  single forwarder this equals `ETX + onward cost`, so Phase 1 agrees with the
  Phase 0 baseline.
- **Destination rule.** Destinations have cost 0, so an outcome that reaches one adds
  no onward cost, which is the report's "total success" rule. When only some
  forwarders receive, the cost is the best receiver's cost. An outcome where every
  forwarder receives takes the best forwarder's cost, not zero.
- **Several destinations** are treated as any-sink: reaching either one counts.
- **Primary cost.** By default a secondary link saves the primary's whole
  opportunistic cost, as the report words it (`--primary-cost total`).
  `--primary-cost airtime` saves only the primary's expected broadcasts, a tighter
  reading that gives smaller gains.
- **Fusion on a forwarding set.** A node gets the discount only if every link in its
  forwarding set is a valid secondary link. For a single link this is exactly
  `Effective_ETX = max(0, ETX - cost of P)`.

## About the numbers

The report's headline result, a drop from 12.4667 to 5.9333 (about 52%) on its
9-node mesh, came from the authors' original code and link probabilities. The
report does not list those probabilities, so this repo cannot reproduce that
exact figure. `topologies/mesh9.txt` is a different 9-node mesh built to contain
the same hidden terminal situation, and its results are the ones shown above. Do
not compare the two sets of numbers.

## Limitations

- Interference is modelled as "neighbours in the link graph", not signal-to-noise
  ratios or a physical channel model.
- Reception probabilities are static.
- The 2^N enumeration caps a forwarding set at 12 nodes, and the pairwise search is
  exhaustive over at most 12 candidate nodes, so it suits small topologies.
- It computes expected routing cost on a graph. It is not a packet-level simulator.

## Layout

```
include/srr/   network.hpp, routing.hpp
src/           network.cpp (parser), routing.cpp (algorithms), main.cpp (CLI)
tests/         test_main.cpp (hand-calculated checks, no framework)
topologies/    linear7.txt, mesh7.txt, mesh9.txt
```

Licensed under the MIT License.
