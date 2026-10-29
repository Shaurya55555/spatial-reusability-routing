#pragma once

#include <limits>
#include <utility>
#include <vector>

#include "srr/network.hpp"

namespace srr {

constexpr double kInf = std::numeric_limits<double>::infinity();
// A forwarding set is enumerated over all 2^|F| reception outcomes, so cap it.
constexpr int kMaxForwarders = 12;

// ---- Phase 0: ETX baseline ------------------------------------------------

// Sum of link ETX along a fixed path (the linear-topology baseline).
double linearCost(const Network& net, const std::vector<int>& path);
// ETX distance from every node to the nearest destination (Dijkstra).
std::vector<double> etxDistances(const Network& net);

// ---- Phase 1: opportunistic cost (EATX) -----------------------------------

struct SetEval {
    double cost;      // attempts + onward
    double attempts;  // expected number of broadcasts until someone receives
    double onward;    // expected cost remaining from the best receiver
};

// Expected cost for node u to broadcast to forwarding set F, enumerating all
// 2^|F| reception outcomes. cost[v] is each forwarder's cost to a destination.
//
//   P(S)    probability that exactly the subset S of F receives
//   fail    P(empty subset): nobody hears the broadcast, so u retries
//   cost(u) = (1 + sum over non-empty S of P(S) * min_{v in S} cost[v]) / (1 - fail)
//
// A destination has cost 0, so an outcome that reaches the destination adds no
// onward cost. With one forwarder this reduces to ETX(u,v) + cost[v].
SetEval evalForwardingSet(const Network& net, int u, const std::vector<int>& F,
                          const std::vector<double>& cost);

// ---- Phase 2/3: spatial reuse ---------------------------------------------

enum class Validation {
    SenderOnly,    // Phase 2 manual rule: only the secondary's sender is checked
    Strict,        // Phase 3 rule from the report: sender AND receiver must both be safe
                   // (each secondary link is checked against the primary only)
    StrictPairwise // extension: also require the masked links to be compatible with
                   // each other, so simultaneous secondaries do not collide among themselves
};

// Can the link u -> v transmit while `primary` transmits?
//   Safe sender:   u is not a neighbour of the primary
//   Safe receiver: v is not a neighbour of the primary (Strict only)
// The primary itself is never part of a secondary link.
bool validSecondary(const Network& net, int primary, int u, int v, Validation mode);

// Can these transmissions happen in the same slot? Links are (sender, receiver)
// pairs; links from the same sender are one broadcast and never conflict with
// each other. Different senders conflict if any of their nodes coincide or are
// neighbours (this covers sender/sender, receiver/receiver and cross collisions).
bool linksCompatible(const Network& net, const std::vector<std::pair<int, int>>& links);

enum class PrimaryCost {
    Total,   // the primary's full opportunistic cost to a destination (as in the report)
    Airtime  // only the expected broadcasts the primary spends (a tighter reading)
};

// "Cost fusion": a secondary node's transmission effort is reduced by the
// primary's cost, never below zero:
//   effective = max(0, attempts - discount)
// For a single link this is exactly Effective_ETX = max(0, ETX - cost of P).
struct Fusion {
    int primary;
    double discount;
    Validation mode;
    const std::vector<bool>* allowed = nullptr;  // if set, only these nodes may be fused
};

struct Solution {
    std::vector<double> cost;              // cost to a destination, per node
    std::vector<std::vector<int>> forwarders;  // chosen forwarding set, per node
    std::vector<double> attempts;          // expected broadcasts at each node
    std::vector<bool> fused;               // node's cost used the fusion discount
    int iterations = 0;
};

// Bellman-Ford-style iterative cost propagation (not plain Dijkstra, because a
// node's cost depends on a whole forwarding set). Costs start at infinity
// (destinations at 0) and are relaxed round by round until nothing improves.
// With `fusion` set, a node whose entire forwarding set consists of valid
// secondary links gets the fusion discount.
Solution solve(const Network& net, const Fusion* fusion = nullptr);

struct Scenario {
    int primary = -1;  // -1 means "no fusion"
    double discount = 0.0;
    double cost = kInf;  // cost at the source
    std::vector<int> fusedNodes;
    std::vector<std::pair<int, int>> maskedLinks;  // links whose cost was masked
    bool violatesStrict = false;    // some masked link fails the strict two-check
    bool violatesPairwise = false;  // some masked links collide with each other
};

struct Report {
    Solution base;
    double baseCost = kInf;
    Scenario best;
    std::vector<Scenario> all;
    double reductionPct = 0.0;
};

// Dynamic scenario tester: base run with no fusion, then every node in turn as
// the Primary, scanning for valid secondary links, applying the fusion
// deduction, and keeping the scenario with the lowest cost at the source.
//
// With Validation::StrictPairwise the tester also searches over which nodes may
// be fused, so the masked links never collide with each other. That search is
// exhaustive over the nodes that have a valid secondary link, and throws
// std::invalid_argument if more than kMaxPairwiseCandidates of them exist.
constexpr int kMaxPairwiseCandidates = 12;
Report runScenarioTester(const Network& net, Validation mode,
                         PrimaryCost primaryCost = PrimaryCost::Total);

}  // namespace srr
