#ifndef FLOW_CONTEST_GOLDBERG_ALGORITHM_H
#define FLOW_CONTEST_GOLDBERG_ALGORITHM_H

#include <list>
#include <cassert>
#include "network.h"

class GoldbergAlgorithm {

private:
    std::vector<size_t> vertex_height_;
    std::vector<NFlow::TFlow> vertex_overflow_;
    std::vector<NFlow::Network::EdgeIterator> outgoing_edge_;
    NFlow::Network *network_ptr_ = nullptr;


    void InitializePreflow() {
        NFlow::Network& network = *network_ptr_;
        vertex_overflow_.assign(network.getVertexNumber(), 0);
        vertex_height_.assign(network.getVertexNumber(), 0);
        vertex_height_[network.getSource()] = network.getVertexNumber();


        for (size_t v = 0; v < network.getVertexNumber(); ++v) {
            for (auto it = network.getListBegin(v); it.isValid(); it.goNext()) {
                it.pushFlow(-it.getFlow());
            }
        }

        for (auto iter = network.getListBegin(network.getSource()); iter.isValid(); iter.goNext()) {
            NFlow::TFlow flow = iter.getResidualCapacity();
            iter.pushFlow(flow);
            vertex_overflow_[network.getSource()] -= flow;
            vertex_overflow_[iter.getFinish()] += flow;
        }

        outgoing_edge_.clear();
        outgoing_edge_.reserve(network.getVertexNumber());
        for (size_t v = 0; v < network.getVertexNumber(); ++v) {
            outgoing_edge_.push_back(network.getListBegin(v));
        }
    }


    void Relabel(NFlow::TVertex vertex) {
        assert(vertex_overflow_[vertex] > 0LL);
        size_t new_height = 1e9;
        for (auto it = network_ptr_->getListBegin(vertex); it.isValid(); it.goNext()) {
            if (it.getResidualCapacity() > 0LL && vertex_height_[it.getFinish()] >= vertex_height_[vertex]) {
                new_height = std::min(new_height, vertex_height_[it.getFinish()]);
            }
        }
        assert(new_height != 1e9);
        vertex_height_[vertex] = new_height + 1;
    }


    void Push(NFlow::Network::EdgeIterator& edgeIterator) {
        assert(vertex_overflow_[edgeIterator.getStart()] > 0);
        assert(edgeIterator.getResidualCapacity() > 0);

        NFlow::TFlow power_flow = std::min(vertex_overflow_[edgeIterator.getStart()], edgeIterator.getResidualCapacity());
        edgeIterator.pushFlow(power_flow);
        vertex_overflow_[edgeIterator.getStart()] -= power_flow;
        vertex_overflow_[edgeIterator.getFinish()] += power_flow;
    }


    void Discharge(NFlow::TVertex vertex) {
        while (vertex_overflow_[vertex] > 0) {
            auto edge_tmp = outgoing_edge_[vertex];
            if (!edge_tmp.isValid()) {
                Relabel(vertex);
                outgoing_edge_[vertex] = network_ptr_->getListBegin(vertex);
            } else if (edge_tmp.getResidualCapacity() > 0 &&
                       vertex_height_[edge_tmp.getStart()] == vertex_height_[edge_tmp.getFinish()] + 1) {
                Push(outgoing_edge_[vertex]);
            } else {
                outgoing_edge_[vertex].goNext();
            }
        }
    }


    GoldbergAlgorithm() = default;

public:

    GoldbergAlgorithm(const GoldbergAlgorithm&) = delete;
    GoldbergAlgorithm& operator =(const GoldbergAlgorithm&) = delete;

    static GoldbergAlgorithm& getInstance() {
        static GoldbergAlgorithm instance;
        return instance;
    }

    void relabelToFront(NFlow::Network& network) {

        NFlow::RemoveAllEdgesDuplicates(network);
        network_ptr_ = &network;
        InitializePreflow();

        std::list<NFlow::TVertex> vertices_list;
        for (size_t v = 0; v < network.getVertexNumber(); ++v) {
            if (v != network.getSource() && v != network.getSink()) {
                vertices_list.push_back(v);
            }
        }

        auto vertices_iterator = vertices_list.begin();
        while (vertices_iterator != vertices_list.end()) {
            int vertex = *vertices_iterator;
            size_t old_vertex_height_ = vertex_height_[vertex];
            Discharge(vertex);
            if (vertex_height_[vertex] > old_vertex_height_) {
                vertices_list.erase(vertices_iterator);
                vertices_list.push_front(vertex);
                vertices_iterator = vertices_list.begin();
            } else {
                ++vertices_iterator;
            }
        }

    }
};

#endif