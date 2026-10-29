#include "srr/network.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace srr {

Network::Network(int nodes) : n_(nodes), p_(nodes, std::vector<double>(nodes, 0.0)) {
    if (nodes < 0) throw std::invalid_argument("node count must be non-negative");
}

void Network::checkNode(int v) const {
    if (v < 0 || v >= n_) throw std::out_of_range("node " + std::to_string(v) + " is out of range");
}

void Network::addLink(int u, int v, double p, bool bidirectional) {
    checkNode(u);
    checkNode(v);
    if (u == v) throw std::invalid_argument("self links are not allowed");
    if (!(p > 0.0 && p <= 1.0)) throw std::invalid_argument("reception probability must be in (0, 1]");
    p_[u][v] = p;
    if (bidirectional) p_[v][u] = p;
}

void Network::setSource(int s) {
    checkNode(s);
    source_ = s;
}

void Network::addDestination(int d) {
    checkNode(d);
    if (!isDestination(d)) dests_.push_back(d);
}

bool Network::isDestination(int v) const {
    return std::find(dests_.begin(), dests_.end(), v) != dests_.end();
}

double Network::reception(int u, int v) const {
    checkNode(u);
    checkNode(v);
    return p_[u][v];
}

double Network::etx(int u, int v) const {
    const double p = reception(u, v);
    return p > 0.0 ? 1.0 / p : std::numeric_limits<double>::infinity();
}

bool Network::interferes(int a, int b) const {
    checkNode(a);
    checkNode(b);
    return a != b && (p_[a][b] > 0.0 || p_[b][a] > 0.0);
}

Network Network::parse(std::istream& in) {
    Network net;
    bool haveNodes = false;
    std::string line;
    int lineNo = 0;
    auto fail = [&](const std::string& why) {
        throw std::runtime_error("topology line " + std::to_string(lineNo) + ": " + why);
    };
    while (std::getline(in, line)) {
        ++lineNo;
        if (const auto hash = line.find('#'); hash != std::string::npos) line.erase(hash);
        std::istringstream ls(line);
        std::string word;
        if (!(ls >> word)) continue;
        if (word == "nodes") {
            int n;
            if (!(ls >> n) || n <= 0) fail("expected a positive node count");
            net = Network(n);
            haveNodes = true;
            continue;
        }
        if (!haveNodes) fail("'nodes N' must come first");
        try {
            if (word == "source") {
                int s;
                if (!(ls >> s)) fail("expected a node id");
                net.setSource(s);
            } else if (word == "dest") {
                int d;
                bool any = false;
                while (ls >> d) {
                    net.addDestination(d);
                    any = true;
                }
                if (!any) fail("expected at least one node id");
            } else if (word == "link" || word == "dlink") {
                int u, v;
                double p;
                if (!(ls >> u >> v >> p)) fail("expected: " + word + " U V P");
                net.addLink(u, v, p, word == "link");
            } else {
                fail("unknown directive '" + word + "'");
            }
        } catch (const std::logic_error& e) {  // out_of_range / invalid_argument
            fail(e.what());
        }
    }
    if (!haveNodes) throw std::runtime_error("topology has no 'nodes' line");
    if (net.destinations().empty()) throw std::runtime_error("topology has no destination");
    return net;
}

Network Network::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open topology file: " + path);
    return parse(f);
}

}  // namespace srr
