// Command-line runner: loads a topology and prints each phase of the analysis.
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "srr/network.hpp"
#include "srr/routing.hpp"

using namespace srr;

namespace {

void usage() {
    std::fprintf(stderr,
                 "usage: spatial_routing <topology.txt> [--primary-cost total|airtime] [--all]\n"
                 "  --primary-cost  how much of the primary's cost a secondary link saves\n"
                 "                  (total = report's rule, airtime = tighter reading)\n"
                 "  --all           list every primary scenario, not just the best\n");
}

std::string linkList(const Scenario& s) {
    if (s.maskedLinks.empty()) return "none";
    std::string out;
    for (const auto& [u, v] : s.maskedLinks) {
        if (!out.empty()) out += ", ";
        out += std::to_string(u) + "->" + std::to_string(v);
    }
    return out;
}

std::string nodeList(const std::vector<int>& v) {
    std::string out;
    for (int x : v) out += (out.empty() ? "" : " ") + std::to_string(x);
    return out;
}

void printScenario(const char* label, const Report& r) {
    std::printf("  %-22s cost %.4f", label, r.best.cost);
    if (r.best.primary < 0) {
        std::printf("  (no beneficial fusion found)\n");
        return;
    }
    std::printf("  (%.1f%% lower)  primary %d, masked links: %s\n", r.reductionPct, r.best.primary,
                linkList(r.best).c_str());
}

}  // namespace

int main(int argc, char** argv) {
    std::string path;
    PrimaryCost pc = PrimaryCost::Total;
    bool listAll = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--all") {
            listAll = true;
        } else if (a == "--primary-cost" && i + 1 < argc) {
            const std::string v = argv[++i];
            if (v == "total") pc = PrimaryCost::Total;
            else if (v == "airtime") pc = PrimaryCost::Airtime;
            else { usage(); return 2; }
        } else if (!a.empty() && a[0] != '-' && path.empty()) {
            path = a;
        } else {
            usage();
            return 2;
        }
    }
    if (path.empty()) {
        usage();
        return 2;
    }

    try {
        const Network net = Network::load(path);
        std::printf("Topology %s: %d nodes, source %d, destinations %s\n\n", path.c_str(),
                    net.size(), net.source(), nodeList(net.destinations()).c_str());

        const double etx = etxDistances(net)[net.source()];
        const Report strict = runScenarioTester(net, Validation::Strict, pc);
        const Report loose = runScenarioTester(net, Validation::SenderOnly, pc);
        const double opp = strict.baseCost;

        std::printf("Phase 0  ETX shortest path          cost %.4f\n", etx);
        std::printf("Phase 1  Opportunistic (EATX)       cost %.4f  (%.1f%% lower than ETX)\n", opp,
                    etx > 0 ? (etx - opp) / etx * 100.0 : 0.0);
        std::printf("         converged in %d Bellman-Ford rounds\n\n", strict.base.iterations);

        std::printf("Phase 2/3  Spatial fusion (primary cost: %s)\n",
                    pc == PrimaryCost::Total ? "total" : "airtime");
        printScenario("sender-only check:", loose);
        printScenario("strict two-check:", strict);
        if (loose.best.violatesStrict)
            std::printf("  -> the sender-only winner masks a link whose receiver is a neighbour of\n"
                        "     the primary: the hidden terminal problem. The strict rule rejects it.\n");
        try {
            const Report pairwise = runScenarioTester(net, Validation::StrictPairwise, pc);
            printScenario("strict + pairwise:", pairwise);
            if (strict.best.violatesPairwise)
                std::printf("  -> the strict winner masks links that collide with each other, so its\n"
                            "     cost is optimistic. The pairwise search keeps only compatible links.\n");
        } catch (const std::invalid_argument& e) {
            std::printf("  strict + pairwise:     skipped (%s)\n", e.what());
        }

        if (listAll) {
            std::printf("\nAll strict scenarios (base cost %.4f):\n", strict.baseCost);
            for (const auto& sc : strict.all)
                std::printf("  primary %d  discount %.4f  source cost %.4f  masked: %s\n", sc.primary,
                            sc.discount, sc.cost, linkList(sc).c_str());
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
