#ifndef FLOW_CONTEST_DINICA_ALGORITHM_H
#define FLOW_CONTEST_DINICA_ALGORITHM_H

#include "network.h"
#include <queue>

std::vector<int> vertex_depth;
std::vector<NFlow::Network::EdgeIterator> edge_iterator;
const NFlow::TFlow INF = 1000000LL;

bool existBlockingFlow(NFlow::Network& network) {
    std::queue<NFlow::TVertex> visited_vertex;
    visited_vertex.push(network.getSource());
    vertex_depth.assign(network.getVertexNumber(), -1);
    vertex_depth[static_cast<size_t>(network.getSource())] = 0;

    while (!visited_vertex.empty()) {
        auto vertex_st = visited_vertex.front();
        visited_vertex.pop();

        for (auto edges_iter = network.getListBegin(vertex_st); edges_iter.isValid(); edges_iter.goNext()) {
            auto vertex_fin = edges_iter.getFinish();
            if (vertex_depth[static_cast<size_t>(vertex_fin)] == -1 && edges_iter.getResidualCapacity() > 0LL) {
                visited_vertex.push(vertex_fin);
                vertex_depth[vertex_fin] = vertex_depth[vertex_st] + 1;
            }
        }
    }

    return vertex_depth[static_cast<size_t>(network.getSink())] != -1;
}

NFlow::TFlow updateBlockingFlow(const NFlow::TVertex& v, const NFlow::TFlow& flow, NFlow::Network& network) {
    if (flow == static_cast<NFlow::TFlow>(0)) {
        return 0;
    } else if (v == network.getSink()) {
        return flow;
    }

    for (; edge_iterator[v].isValid(); edge_iterator[v].goNext()) {
        NFlow::TVertex u = edge_iterator[v].getFinish();
        if (vertex_depth[u] == vertex_depth[v] + 1) {
            auto summary_flow = updateBlockingFlow(u, std::min(flow, edge_iterator[v].getResidualCapacity()), network);
            if (summary_flow != static_cast<NFlow::TFlow>(0)) {
                edge_iterator[v].pushFlow(summary_flow);
                return summary_flow;
            }
        }
    }

    return 0;
}

void dinica_algorithm(NFlow::Network& network) {
    while (existBlockingFlow(network)) {
        edge_iterator.clear();
        edge_iterator.reserve(network.getVertexNumber());
        for (size_t i = 0; i < network.getVertexNumber(); ++i) {
            edge_iterator.push_back(network.getListBegin(static_cast<NFlow::TVertex>(i)));
        }
        while (updateBlockingFlow(network.getSource(), INF, network) != static_cast<NFlow::TFlow>(0)) {}
    }
}

#endif //FLOW_CONTEST_DINICA_ALGORITHM_H
