#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace srr {

// A multi-hop wireless network. Every directed link carries a reception
// probability p, so its ETX (expected transmission count) is 1/p.
//
// Two nodes "interfere" (are neighbours) when a link exists between them in
// either direction. That relation drives the spatial-reuse validity checks.
class Network {
public:
    explicit Network(int nodes = 0);

    int size() const { return n_; }

    // p must be in (0, 1]. A bidirectional link sets both directions to p.
    void addLink(int u, int v, double p, bool bidirectional = true);
    void setSource(int s);
    void addDestination(int d);

    int source() const { return source_; }
    const std::vector<int>& destinations() const { return dests_; }
    bool isDestination(int v) const;

    // 0 when there is no link u -> v.
    double reception(int u, int v) const;
    // 1/p, or +infinity when there is no link.
    double etx(int u, int v) const;
    // a and b are neighbours: a link exists between them in either direction.
    bool interferes(int a, int b) const;

    // Topology file format (one directive per line, '#' starts a comment):
    //   nodes N
    //   source S
    //   dest D [D ...]
    //   link U V P     symmetric link with reception probability P
    //   dlink U V P    directed link U -> V
    // Throws std::runtime_error on malformed input.
    static Network parse(std::istream& in);
    static Network load(const std::string& path);

private:
    void checkNode(int v) const;

    int n_ = 0;
    int source_ = 0;
    std::vector<int> dests_;
    std::vector<std::vector<double>> p_;
};

}  // namespace srr
