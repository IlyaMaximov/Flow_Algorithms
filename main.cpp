#include "dinica_algorithm.h"
#include "goldberg_algorithm.h"
#include "MKM_algorithm.h"
#include <iostream>


int main() {
    const NFlow::TFlow INF = 1e17;

    size_t vertexNumber;
    std::cin >> vertexNumber;
    std::vector<NFlow::TFlow> cost(vertexNumber);
    for (size_t vertex = 0; vertex < vertexNumber; ++vertex) {
        std::cin >> cost[vertex];
    }


    NFlow::TVertex source = vertexNumber;
    NFlow::TVertex  sink = vertexNumber + 1;
    NFlow::Network network1(vertexNumber + 2, source, sink);
    for (size_t vertex_st = 0; vertex_st < vertexNumber; ++vertex_st) {
        size_t ver_cnt;
        std::cin >> ver_cnt;
        for (size_t i = 0; i < ver_cnt; ++i) {
            size_t vertex_fin;
            std::cin >> vertex_fin;
            network1.addEdge(vertex_st, --vertex_fin, INF);
        }
    }

    NFlow::TFlow ans = 0LL;
    for (size_t vertex = 0; vertex < vertexNumber; ++vertex) {
        if (cost[vertex] > 0LL) {
            ans += cost[vertex];
            network1.addEdge(source, vertex, cost[vertex]);
        } else {
            network1.addEdge(vertex, sink, -cost[vertex]);
        }
    }


    NFlow::Network network2 = network1;
    MKM_Algorithm::MKM_algorithm(network2);

    GoldbergAlgorithm& algorithm = GoldbergAlgorithm::getInstance();
    algorithm.relabelToFront(network1);

    assert(NFlow::getFlowValue(network1) == NFlow::getFlowValue(network2));
    std::cout << ans - NFlow::getFlowValue(network1);

    return 0;
}