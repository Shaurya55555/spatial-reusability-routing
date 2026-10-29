#include "srr/routing.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace srr {

namespace {
constexpr double kEps = 1e-12;
}

double linearCost(const Network& net, const std::vector<int>& path) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const double e = net.etx(path[i], path[i + 1]);
        if (!std::isfinite(e)) return kInf;
        total += e;
    }
    return total;
}

std::vector<double> etxDistances(const Network& net) {
    const int n = net.size();
    std::vector<double> dist(n, kInf);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    for (int d : net.destinations()) {
        dist[d] = 0.0;
        pq.push({0.0, d});
    }
    while (!pq.empty()) {
        const auto [dv, v] = pq.top();
        pq.pop();
        if (dv > dist[v]) continue;
        for (int u = 0; u < n; ++u) {
            if (net.reception(u, v) <= 0.0) continue;
            const double nd = dv + net.etx(u, v);
            if (nd < dist[u]) {
                dist[u] = nd;
                pq.push({nd, u});
            }
        }
    }
    return dist;
}

SetEval evalForwardingSet(const Network& net, int u, const std::vector<int>& F,
                          const std::vector<double>& cost) {
    const size_t k = F.size();
    if (k == 0 || k > static_cast<size_t>(kMaxForwarders))
        throw std::invalid_argument("forwarding set size must be between 1 and kMaxForwarders");

    double fail = 0.0;
    double weighted = 0.0;  // sum over non-empty S of P(S) * min cost in S
    for (unsigned mask = 0; mask < (1u << k); ++mask) {
        double pr = 1.0;
        double best = kInf;
        for (size_t i = 0; i < k; ++i) {
            const double p = net.reception(u, F[i]);
            if (mask & (1u << i)) {
                pr *= p;
                best = std::min(best, cost[F[i]]);
            } else {
                pr *= 1.0 - p;
            }
        }
        if (mask == 0)
            fail = pr;
        else if (pr > 0.0)
            weighted += pr * best;
    }
    const double success = 1.0 - fail;
    if (success <= kEps) return {kInf, kInf, kInf};
    const double attempts = 1.0 / success;
    const double onward = weighted / success;
    return {attempts + onward, attempts, onward};
}

bool validSecondary(const Network& net, int primary, int u, int v, Validation mode) {
    if (u == primary || v == primary) return false;
    if (net.reception(u, v) <= 0.0) return false;
    if (net.interferes(u, primary)) return false;                          // safe sender
    if (mode != Validation::SenderOnly && net.interferes(v, primary)) return false;  // safe receiver
    return true;
}

bool linksCompatible(const Network& net, const std::vector<std::pair<int, int>>& links) {
    // Group the links by sender: one sender's links are a single broadcast.
    std::vector<std::vector<int>> groups;  // each group: sender followed by its receivers
    std::vector<int> senders;
    for (const auto& [u, v] : links) {
        auto it = std::find(senders.begin(), senders.end(), u);
        if (it == senders.end()) {
            senders.push_back(u);
            groups.push_back({u});
            groups.back().push_back(v);
        } else {
            groups[it - senders.begin()].push_back(v);
        }
    }
    for (size_t i = 0; i < groups.size(); ++i)
        for (size_t j = i + 1; j < groups.size(); ++j)
            for (int x : groups[i])
                for (int y : groups[j])
                    if (x == y || net.interferes(x, y)) return false;
    return true;
}

Solution solve(const Network& net, const Fusion* fusion) {
    const int n = net.size();
    Solution s;
    s.cost.assign(n, kInf);
    s.forwarders.assign(n, {});
    s.attempts.assign(n, kInf);
    s.fused.assign(n, false);
    for (int d : net.destinations()) {
        s.cost[d] = 0.0;
        s.attempts[d] = 0.0;
    }

    for (int iter = 1; iter <= 2 * n + 2; ++iter) {
        s.iterations = iter;
        bool changed = false;
        const std::vector<double> prev = s.cost;  // relax from the previous round

        for (int u = 0; u < n; ++u) {
            if (net.isDestination(u)) continue;

            // Candidate forwarders: neighbours already closer to a destination.
            std::vector<int> cand;
            for (int v = 0; v < n; ++v)
                if (net.reception(u, v) > 0.0 && std::isfinite(prev[v]) && prev[v] < prev[u])
                    cand.push_back(v);
            std::sort(cand.begin(), cand.end(), [&](int a, int b) {
                return prev[a] != prev[b] ? prev[a] < prev[b] : a < b;
            });
            if (cand.size() > static_cast<size_t>(kMaxForwarders)) cand.resize(kMaxForwarders);

            double bestCost = kInf, bestAttempts = kInf;
            std::vector<int> bestSet;
            bool bestFused = false;
            // The best forwarding set is a prefix of the candidates sorted by cost.
            for (size_t k = 1; k <= cand.size(); ++k) {
                const std::vector<int> F(cand.begin(), cand.begin() + k);
                SetEval e = evalForwardingSet(net, u, F, prev);
                bool fused = false;
                if (fusion) {
                    fused = (!fusion->allowed || (*fusion->allowed)[u]) &&
                            std::all_of(F.begin(), F.end(), [&](int v) {
                                return validSecondary(net, fusion->primary, u, v, fusion->mode);
                            });
                    if (fused) {
                        e.attempts = std::max(0.0, e.attempts - fusion->discount);
                        e.cost = e.attempts + e.onward;
                    }
                }
                if (e.cost < bestCost - kEps) {
                    bestCost = e.cost;
                    bestAttempts = e.attempts;
                    bestSet = F;
                    bestFused = fused;
                }
            }
            if (bestCost < s.cost[u] - kEps) {
                s.cost[u] = bestCost;
                s.attempts[u] = bestAttempts;
                s.forwarders[u] = bestSet;
                s.fused[u] = bestFused;
                changed = true;
            }
        }
        if (!changed) break;
    }
    return s;
}

namespace {

// Nodes the source actually relies on, following the chosen forwarding sets.
std::vector<int> reachableFromSource(const Network& net, const Solution& s) {
    std::vector<bool> seen(net.size(), false);
    std::vector<int> order;
    std::queue<int> q;
    q.push(net.source());
    seen[net.source()] = true;
    while (!q.empty()) {
        const int u = q.front();
        q.pop();
        order.push_back(u);
        for (int f : s.forwarders[u])
            if (!seen[f]) {
                seen[f] = true;
                q.push(f);
            }
    }
    return order;
}

Scenario describe(const Network& net, const Solution& s, int primary, double discount) {
    Scenario sc;
    sc.primary = primary;
    sc.discount = discount;
    sc.cost = s.cost[net.source()];
    for (int u : reachableFromSource(net, s)) {
        if (!s.fused[u]) continue;
        sc.fusedNodes.push_back(u);
        for (int v : s.forwarders[u]) {
            sc.maskedLinks.emplace_back(u, v);
            if (!validSecondary(net, primary, u, v, Validation::Strict)) sc.violatesStrict = true;
        }
    }
    sc.violatesPairwise = !linksCompatible(net, sc.maskedLinks);
    return sc;
}

}  // namespace

Report runScenarioTester(const Network& net, Validation mode, PrimaryCost primaryCost) {
    Report r;
    r.base = solve(net, nullptr);  // base run: zero fusions
    r.baseCost = r.base.cost[net.source()];
    r.best.cost = r.baseCost;

    for (int p = 0; p < net.size(); ++p) {  // every node is a potential Primary
        if (net.isDestination(p) || !std::isfinite(r.base.cost[p])) continue;
        const double discount =
            primaryCost == PrimaryCost::Total ? r.base.cost[p] : r.base.attempts[p];

        Scenario sc;
        if (mode != Validation::StrictPairwise) {
            const Fusion f{p, discount, mode};
            sc = describe(net, solve(net, &f), p, discount);
        } else {
            // Nodes that could be fused: those with at least one valid secondary link.
            std::vector<int> candidates;
            for (int u = 0; u < net.size(); ++u) {
                if (u == p || net.isDestination(u)) continue;
                for (int v = 0; v < net.size(); ++v)
                    if (validSecondary(net, p, u, v, Validation::Strict)) {
                        candidates.push_back(u);
                        break;
                    }
            }
            if (candidates.size() > static_cast<size_t>(kMaxPairwiseCandidates))
                throw std::invalid_argument("too many candidate nodes for the exhaustive pairwise search");

            // Try every subset of nodes allowed to be fused; keep the cheapest
            // outcome whose masked links do not collide with each other.
            sc.primary = p;
            sc.discount = discount;
            sc.cost = r.baseCost;  // the empty subset: no fusion
            for (unsigned mask = 1; mask < (1u << candidates.size()); ++mask) {
                std::vector<bool> allowed(net.size(), false);
                for (size_t i = 0; i < candidates.size(); ++i)
                    if (mask & (1u << i)) allowed[candidates[i]] = true;
                const Fusion f{p, discount, mode, &allowed};
                Scenario trial = describe(net, solve(net, &f), p, discount);
                if (!trial.violatesPairwise && trial.cost < sc.cost - kEps) sc = trial;
            }
        }
        r.all.push_back(sc);
        if (sc.cost < r.best.cost - kEps) r.best = sc;
    }

    if (std::isfinite(r.baseCost) && r.baseCost > 0.0)
        r.reductionPct = (r.baseCost - r.best.cost) / r.baseCost * 100.0;
    return r;
}

}  // namespace srr
