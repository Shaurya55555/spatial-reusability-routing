// Dependency-free tests. Run with `ctest` or ./srr_tests.
#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "srr/network.hpp"
#include "srr/routing.hpp"

using namespace srr;

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++checks;                                                                \
        if (!(cond)) {                                                           \
            ++failures;                                                          \
            std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                        \
    } while (0)

static bool near(double a, double b, double eps = 1e-9) { return std::fabs(a - b) < eps; }

#ifndef TOPOLOGY_DIR
#define TOPOLOGY_DIR "topologies"
#endif
static Network topo(const char* name) { return Network::load(std::string(TOPOLOGY_DIR) + "/" + name); }

// Phase 0: the linear baseline is the plain sum of link ETX.
static void testLinearBaseline() {
    const Network net = topo("linear7.txt");
    const double expected = 1 / 0.9 + 1 / 0.8 + 1 / 0.7 + 1 / 0.9 + 1 / 0.6 + 1 / 0.8;
    CHECK(near(linearCost(net, {0, 1, 2, 3, 4, 5, 6}), expected));
    CHECK(near(etxDistances(net)[0], expected));
    // With no choice of next hop, opportunistic routing cannot beat ETX.
    CHECK(near(solve(net).cost[0], expected));
    CHECK(std::isinf(linearCost(net, {0, 2})));  // no link 0 -> 2
}

// Phase 1: the 2^N reception-outcome cost, checked by hand.
static void testOpportunisticCost() {
    // One forwarder reduces to ETX + onward cost.
    Network one(2);
    one.addLink(0, 1, 0.5);
    one.addDestination(1);
    const std::vector<double> c1 = {kInf, 0.0};
    CHECK(near(evalForwardingSet(one, 0, {1}, c1).cost, 2.0));

    // Two destination forwarders, p = 0.5 each: P(nobody hears) = 0.25, so
    // 1 / 0.75 expected broadcasts and nothing onward.
    Network two(3);
    two.addLink(0, 1, 0.5);
    two.addLink(0, 2, 0.5);
    two.addDestination(1);
    two.addDestination(2);
    const std::vector<double> c2 = {kInf, 0.0, 0.0};
    const SetEval e = evalForwardingSet(two, 0, {1, 2}, c2);
    CHECK(near(e.cost, 1.0 / 0.75));
    CHECK(near(e.onward, 0.0));

    // Forwarders with onward costs 2 and 4:
    //   outcomes {v1}:.25 min 2, {v2}:.25 min 4, {v1,v2}:.25 min 2, none:.25
    //   (1 + .5 + 1 + .5) / .75 = 4
    const std::vector<double> c3 = {kInf, 2.0, 4.0};
    CHECK(near(evalForwardingSet(two, 0, {1, 2}, c3).cost, 4.0));

    bool threw = false;
    try { evalForwardingSet(two, 0, {}, c3); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
}

// Phase 1: opportunistic routing beats ETX on a mesh, and the iterative solver
// reaches a fixed point.
static void testBellmanFordSolver() {
    const Network net = topo("mesh7.txt");
    const Solution s = solve(net);
    CHECK(s.cost[0] < etxDistances(net)[0] - 1e-9);
    CHECK(s.iterations <= 2 * net.size() + 2);
    for (int u = 0; u < net.size(); ++u) {
        if (net.isDestination(u)) continue;
        CHECK(std::isfinite(s.cost[u]));
        // Fixed point: each node's cost equals the cost of its chosen set.
        CHECK(near(s.cost[u], evalForwardingSet(net, u, s.forwarders[u], s.cost).cost, 1e-9));
        // Costs strictly decrease towards the destination: no routing loops.
        for (int f : s.forwarders[u]) CHECK(s.cost[f] < s.cost[u]);
    }
}

// Phase 3: the strict two-check validation and the hidden terminal problem.
static void testHiddenTerminalValidation() {
    const Network net = topo("mesh9.txt");
    CHECK(!net.interferes(2, 6));  // sender is far from the primary
    CHECK(net.interferes(4, 6));   // but the receiver is a neighbour of it
    CHECK(validSecondary(net, 6, 2, 4, Validation::SenderOnly));  // the naive rule allows it
    CHECK(!validSecondary(net, 6, 2, 4, Validation::Strict));     // the strict rule rejects it
    CHECK(!validSecondary(net, 6, 6, 4, Validation::SenderOnly)); // primary is never a sender
    CHECK(!validSecondary(net, 6, 2, 6, Validation::Strict));     // nor a receiver
    CHECK(validSecondary(net, 6, 0, 1, Validation::Strict));      // 0 -> 1 is clear of 6
    CHECK(!validSecondary(net, 6, 0, 3, Validation::Strict));     // no such link
}

// Phase 2: cost fusion, checked by hand on a tiny network.
static void testFusionDeduction() {
    // Link 0 -> 1 has ETX 2. Primary 2 reaches destination 3 at cost 2, and
    // is far from both 0 and 1, so Effective_ETX = max(0, 2 - 2) = 0.
    Network net(4);
    net.setSource(0);
    net.addDestination(1);
    net.addDestination(3);
    net.addLink(0, 1, 0.5);
    net.addLink(2, 3, 0.5);
    const Report r = runScenarioTester(net, Validation::Strict);
    CHECK(near(r.baseCost, 2.0));
    CHECK(r.best.primary == 2);
    CHECK(near(r.best.cost, 0.0));
    CHECK(r.best.maskedLinks.size() == 1 && r.best.maskedLinks[0] == std::make_pair(0, 1));
    CHECK(near(r.reductionPct, 100.0));

    // The discount never goes below zero, so a bigger discount changes nothing.
    const Fusion big{2, 100.0, Validation::Strict};
    CHECK(near(solve(net, &big).cost[0], 0.0));
}

// Phase 3: the scenario tester on the 9-node mesh.
static void testScenarioTester() {
    const Network net = topo("mesh9.txt");
    for (PrimaryCost pc : {PrimaryCost::Total, PrimaryCost::Airtime}) {
        const Report strict = runScenarioTester(net, Validation::Strict, pc);
        const Report loose = runScenarioTester(net, Validation::SenderOnly, pc);
        CHECK(std::isfinite(strict.baseCost));
        CHECK(near(strict.baseCost, loose.baseCost));
        // Fusion only ever reduces cost, and a looser rule has more options.
        CHECK(strict.best.cost <= strict.baseCost + 1e-9);
        CHECK(loose.best.cost <= strict.best.cost + 1e-9);
        CHECK(strict.reductionPct >= 0.0 && strict.reductionPct <= 100.0);
        // The strict rule never produces a scenario that violates itself.
        for (const auto& sc : strict.all) CHECK(!sc.violatesStrict);
        // Every non-destination node with a finite base cost was tried as primary.
        CHECK(strict.all.size() == 7);
    }
    // Total mode deducts at least as much as airtime mode, so it can't do worse.
    CHECK(runScenarioTester(net, Validation::Strict, PrimaryCost::Total).best.cost <=
          runScenarioTester(net, Validation::Strict, PrimaryCost::Airtime).best.cost + 1e-9);
}

// Extension: masked links must also be compatible with each other.
static void testPairwiseCompatibility() {
    const Network chain = topo("linear7.txt");
    // 2->3 and 3->4 share node 3; 2->3 and 4->5 have neighbouring nodes 3 and 4.
    CHECK(!linksCompatible(chain, {{2, 3}, {3, 4}}));
    CHECK(!linksCompatible(chain, {{2, 3}, {4, 5}}));
    CHECK(linksCompatible(chain, {{0, 1}, {3, 4}}));  // 1 and 3 are two hops apart
    CHECK(linksCompatible(chain, {{2, 3}}));
    CHECK(linksCompatible(chain, {}));

    // The report's rule masks links 2->3, 3->4, 4->5, 5->6 together on a chain;
    // the pairwise rule must reject that and report a smaller gain.
    const Report strict = runScenarioTester(chain, Validation::Strict);
    const Report pairwise = runScenarioTester(chain, Validation::StrictPairwise);
    CHECK(strict.best.violatesPairwise);
    CHECK(!pairwise.best.violatesPairwise);
    CHECK(pairwise.best.cost >= strict.best.cost - 1e-9);
    CHECK(pairwise.best.cost <= pairwise.baseCost + 1e-9);

    const Network mesh = topo("mesh9.txt");
    const Report m = runScenarioTester(mesh, Validation::StrictPairwise);
    CHECK(m.best.cost <= m.baseCost + 1e-9);
    CHECK(m.all.size() == 7);
    CHECK(!m.best.violatesStrict && !m.best.violatesPairwise);
    for (const auto& sc : m.all) CHECK(!sc.violatesStrict && !sc.violatesPairwise);
    CHECK(m.best.cost >= runScenarioTester(mesh, Validation::Strict).best.cost - 1e-9);
}

static void testParser() {
    std::istringstream ok("# demo\nnodes 3\nsource 0\ndest 2\nlink 0 1 0.5\ndlink 1 2 0.8\n");
    const Network net = Network::parse(ok);
    CHECK(net.size() == 3 && net.source() == 0 && net.isDestination(2));
    CHECK(near(net.reception(0, 1), 0.5) && near(net.reception(1, 0), 0.5));
    CHECK(near(net.reception(1, 2), 0.8) && net.reception(2, 1) == 0.0);

    for (const char* bad : {"link 0 1 0.5\n", "nodes 2\ndest 0\nlink 0 5 0.5\n",
                            "nodes 2\ndest 0\nlink 0 1 1.5\n", "nodes 2\nsource 0\n",
                            "nodes 2\ndest 1\nbogus 1\n"}) {
        std::istringstream in(bad);
        bool threw = false;
        try { Network::parse(in); } catch (const std::exception&) { threw = true; }
        CHECK(threw);
    }
}

int main() {
    testLinearBaseline();
    testOpportunisticCost();
    testBellmanFordSolver();
    testHiddenTerminalValidation();
    testFusionDeduction();
    testScenarioTester();
    testPairwiseCompatibility();
    testParser();
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
