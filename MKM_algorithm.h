#ifndef FLOW_CONTEST_MKM_HELPER_GRAPH
#define FLOW_CONTEST_MKM_HELPER_GRAPH

#include <cassert>
#include <forward_list>
#include <queue>
#include "network.h"


class MKM_Algorithm {

private:
    typedef NFlow::TFlow TFlow;
    typedef NFlow::NInner::TVertexNumber TVertexNumber;
    static const long long INF = 1e16;


    class HelperGraph {
    public:

    class TVertex  {
        private:
            NFlow::TVertex vertex_id_;
            TFlow potential_in_ = 0LL, potential_out_ = 0LL, potential_ = 0LL, deferred_flow_ = 0LL;
            bool is_deleted_ = false;

        public:
            explicit TVertex(const NFlow::TVertex &vertex) : vertex_id_(vertex) {}

            void addPotentialIn(const TFlow val) {
                potential_in_ += val;
                potential_ = std::min(potential_in_, potential_out_);
                assert(potential_ >= 0LL);
            }

            void addPotentialOut(const TFlow val) {
                potential_out_ += val;
                potential_ = std::min(potential_in_, potential_out_);
                assert(potential_ >= 0LL);
            }

            void delVertex() {
                is_deleted_ = true;
            }

            void addDeferredFlow(const TFlow added_flow) {
                deferred_flow_ += added_flow;
            }

            [[nodiscard]] NFlow::TVertex getId() const {
                return vertex_id_;
            }

            [[nodiscard]] NFlow::TFlow getPotential() const {
                return potential_;
            }

            [[nodiscard]] bool isDeleted() const {
                return is_deleted_;
            }

            [[nodiscard]] TFlow getDeferredFlow() const {
                return deferred_flow_;
            }
        };

    private:

        class Edge {
        private:
            TVertex &start_, &finish_;
            TFlow capacity_, flow_;
            NFlow::Network::EdgeIterator network_iterator_;

        public:
            explicit Edge(const NFlow::Network::EdgeIterator &iter, TVertex &start, TVertex &finish)
                    : network_iterator_(iter), start_(start), finish_(finish),
                      capacity_(iter.getCapacity()), flow_(iter.getFlow()) {}

            [[nodiscard]] NFlow::TFlow getResidualCapacity() const {
                return capacity_ - flow_;
            }

            [[nodiscard]] TVertex &getStart() const {
                return start_;
            }

            [[nodiscard]] TVertex &getFinish() const {
                return finish_;
            }

            void pushFlow(const TFlow additional_flow) {
                flow_ += additional_flow;
                network_iterator_.pushFlow(additional_flow);

                start_.addPotentialOut(-additional_flow);
                finish_.addPotentialIn(-additional_flow);
            }
        };


        TVertex *source_, *sink_;
        TVertexNumber vertex_number_;
        std::vector<TVertex> graph_vertex_;
        std::vector<std::forward_list<Edge>> direct_edges_;
        std::vector<std::forward_list<Edge *>> reverse_edges_;
        std::vector<std::forward_list<Edge>::iterator> direct_edges_iterator_;
        std::vector<std::forward_list<Edge *>::iterator> reverse_edges_iterator_;

    public:

        template<typename Func>
        explicit HelperGraph(NFlow::Network &network, Func &is_good_edge) {

            vertex_number_ = network.getVertexNumber();
            graph_vertex_.reserve(vertex_number_);
            for (size_t v = 0; v < vertex_number_; ++v) {
                graph_vertex_.emplace_back(v);
            }
            source_ = &graph_vertex_[network.getSource()];
            sink_ = &graph_vertex_[network.getSink()];


            direct_edges_.resize(vertex_number_);
            reverse_edges_.resize(vertex_number_);
            direct_edges_iterator_.resize(vertex_number_);
            reverse_edges_iterator_.resize(vertex_number_);

            for (size_t v = 0; v < network.getVertexNumber(); ++v) {
                for (auto edge_iter = network.getListBegin(v); edge_iter.isValid(); edge_iter.goNext()) {
                    if (is_good_edge(edge_iter, network)) {

                        TVertex &start = graph_vertex_[edge_iter.getStart()];
                        TVertex &finish = graph_vertex_[edge_iter.getFinish()];

                        direct_edges_[start.getId()].push_front(Edge(edge_iter, start, finish));
                        reverse_edges_[finish.getId()].push_front(&direct_edges_[start.getId()].front());
                    }
                }
            }

            for (size_t v = 0; v < network.getVertexNumber(); ++v) {
                direct_edges_iterator_[v] = direct_edges_[v].begin();
                reverse_edges_iterator_[v] = reverse_edges_[v].begin();
            }
        }

        [[nodiscard]] std::forward_list<Edge>::iterator getDirectEdgesIterator(const size_t id) {
            return direct_edges_iterator_[id];
        }

        void goNextDirectEdgesIterator(const size_t id) {
            ++direct_edges_iterator_[id];
        }

        [[nodiscard]] bool isValidDirectEdgesIterator(const size_t id) const {
            return direct_edges_iterator_[id] != direct_edges_[id].end();
        }

        [[nodiscard]] std::forward_list<Edge *>::iterator getReverseEdgesIterator(const size_t id) {
            return reverse_edges_iterator_[id];
        }

        void goNextReverseEdgesIterator(const size_t id) {
            ++reverse_edges_iterator_[id];
        }

        [[nodiscard]] bool isValidReverseEdgesIterator(const size_t id) const {
            return reverse_edges_iterator_[id] != reverse_edges_[id].end();
        }

        [[nodiscard]] TVertexNumber getVertexNumber() const {
            return vertex_number_;
        }

        [[nodiscard]] TVertex &getSource() const {
            return *source_;
        }

        [[nodiscard]] TVertex &getSink() const {
            return *sink_;
        }

        [[nodiscard]] bool existUpdateBlockingFlow() const {
            return !source_->isDeleted() && !sink_->isDeleted();
        }


        TVertex& getReferenceNode() {
            NFlow::TFlow min_potential = INF;
            TVertex *ref_node = nullptr;

            for (auto &vertex : graph_vertex_) {
                if (!vertex.isDeleted() && vertex.getPotential() < min_potential) {
                    assert(vertex.getPotential() != 0LL);
                    ref_node = &vertex;
                    min_potential = vertex.getPotential();
                }
            }

            if (ref_node == nullptr) {
                throw std::runtime_error("");
            }
            return *ref_node;
        }

        void findPotentials() {
            getSource().addPotentialIn(INF);
            getSink().addPotentialOut(INF);

            for (NFlow::TVertex v = 0; v < getVertexNumber(); ++v) {
                for (auto &edge: direct_edges_[v]) {
                    edge.getStart().addPotentialOut(edge.getResidualCapacity());
                    edge.getFinish().addPotentialIn(edge.getResidualCapacity());
                }
            }
        }

        void removalAllBlockedVertex() {
            std::queue<TVertex *> blocked_vertex;
            for (auto &vertex : graph_vertex_) {
                if (vertex.getPotential() == 0 && !vertex.isDeleted()) {
                    blocked_vertex.push(&vertex);
                }
            }

            while (!blocked_vertex.empty()) {
                TVertex &vertex = *blocked_vertex.front();
                blocked_vertex.pop();

                while (isValidDirectEdgesIterator(vertex.getId())) {
                    Edge &edge = *getDirectEdgesIterator(vertex.getId());
                    TVertex &finish = edge.getFinish();
                    if (!finish.isDeleted()) {
                        vertex.addPotentialOut(-edge.getResidualCapacity());
                        finish.addPotentialIn(-edge.getResidualCapacity());
                        if (finish.getPotential() == 0LL) {
                            blocked_vertex.push(&finish);
                        }
                    }
                    goNextDirectEdgesIterator(vertex.getId());
                }

                while (isValidReverseEdgesIterator(vertex.getId())) {
                    Edge &edge = *(*getReverseEdgesIterator(vertex.getId()));
                    TVertex &start = edge.getStart();
                    if (!start.isDeleted()) {
                        start.addPotentialOut(-edge.getResidualCapacity());
                        vertex.addPotentialIn(-edge.getResidualCapacity());
                        if (start.getPotential() == 0LL) {
                            blocked_vertex.push(&start);
                        }
                    }
                    goNextReverseEdgesIterator(vertex.getId());
                }

                vertex.delVertex();
            }
        }

    };


    static void PushVertexSinkFlow(HelperGraph::TVertex &ref_vertex, const NFlow::TFlow start_flow, HelperGraph &helper_graph) {
        std::queue<HelperGraph::TVertex *> path;
        ref_vertex.addDeferredFlow(start_flow);
        path.push(&ref_vertex);

        while (!path.empty()) {
            HelperGraph::TVertex &start = *path.front();
            path.pop();
            TFlow flow = start.getDeferredFlow();

            while (flow > 0LL && helper_graph.isValidDirectEdgesIterator(start.getId())) {
                auto &edge = *helper_graph.getDirectEdgesIterator(start.getId());
                if (edge.getFinish().isDeleted() || edge.getResidualCapacity() == 0LL) {
                    helper_graph.goNextDirectEdgesIterator(start.getId());
                    continue;
                }

                NFlow::TFlow power_flow = std::min(edge.getResidualCapacity(), flow);
                edge.pushFlow(power_flow);
                if (edge.getFinish().getDeferredFlow() == 0LL) {
                    path.push(&edge.getFinish());
                }
                edge.getFinish().addDeferredFlow(power_flow);

                flow -= power_flow;
                if (flow > 0LL) {
                    helper_graph.goNextDirectEdgesIterator(start.getId());
                }
            }

            start.addDeferredFlow(-start.getDeferredFlow());
        }
    }


    static void PushSourceVertexFlow(HelperGraph::TVertex &ref_vertex, NFlow::TFlow st_flow, HelperGraph &helper_graph) {
        std::queue<HelperGraph::TVertex *> path;
        ref_vertex.addDeferredFlow(st_flow);
        path.push(&ref_vertex);

        while (!path.empty()) {
            HelperGraph::TVertex &finish = *path.front();
            path.pop();
            TFlow flow = finish.getDeferredFlow();

            while (flow > 0LL && helper_graph.isValidReverseEdgesIterator(finish.getId())) {
                auto &edge = *(*helper_graph.getReverseEdgesIterator(finish.getId()));
                if (edge.getStart().isDeleted() || edge.getResidualCapacity() == 0LL) {
                    helper_graph.goNextReverseEdgesIterator(finish.getId());
                    continue;
                }

                NFlow::TFlow power_flow = std::min(edge.getResidualCapacity(), flow);
                edge.pushFlow(power_flow);
                if (edge.getStart().getDeferredFlow() == 0LL) {
                    path.push(&edge.getStart());
                }
                edge.getStart().addDeferredFlow(power_flow);

                flow -= power_flow;
                if (flow > 0LL) {
                    helper_graph.goNextReverseEdgesIterator(finish.getId());
                }
            }

            finish.addDeferredFlow(-finish.getDeferredFlow());
        }
    }


    static void findSourceVertexDist(NFlow::Network &network, std::vector<int> &source_vertex_dist) {
        std::queue<NFlow::TVertex> visited_vertex;
        source_vertex_dist.assign(network.getVertexNumber(), -1);

        visited_vertex.push(network.getSource());
        source_vertex_dist[static_cast<size_t>(network.getSource())] = 0;

        while (!visited_vertex.empty()) {
            auto vertex_st = visited_vertex.front();
            visited_vertex.pop();

            for (auto edges_iter = network.getListBegin(vertex_st); edges_iter.isValid(); edges_iter.goNext()) {
                auto vertex_fin = edges_iter.getFinish();
                if (source_vertex_dist[static_cast<size_t>(vertex_fin)] == -1 &&
                    edges_iter.getResidualCapacity() > 0LL) {
                    visited_vertex.push(vertex_fin);
                    source_vertex_dist[vertex_fin] = source_vertex_dist[vertex_st] + 1;
                }
            }
        }
    }


    static void findVertexSinkDist(NFlow::Network &network, std::vector<int> &vertex_sink_dist) {
        std::queue<NFlow::TVertex> visited_vertex;
        vertex_sink_dist.assign(network.getVertexNumber(), -1);

        visited_vertex.push(network.getSink());
        vertex_sink_dist[static_cast<size_t>(network.getSink())] = 0;

        while (!visited_vertex.empty()) {
            auto vertex_fin = visited_vertex.front();
            visited_vertex.pop();

            for (auto edges_iter_ = network.getListBegin(vertex_fin); edges_iter_.isValid(); edges_iter_.goNext()) {
                auto edges_iter = edges_iter_.reverse_edge();
                auto vertex_st = edges_iter.getStart();
                if (vertex_sink_dist[static_cast<size_t>(vertex_st)] == -1 && edges_iter.getResidualCapacity() > 0LL) {
                    visited_vertex.push(vertex_st);
                    vertex_sink_dist[vertex_st] = vertex_sink_dist[vertex_fin] + 1;
                }
            }
        }
    }


public:

    MKM_Algorithm() = delete;
    MKM_Algorithm(const MKM_Algorithm&) = delete;
    MKM_Algorithm& operator =(const MKM_Algorithm&) = delete;


    static void MKM_algorithm(NFlow::Network &network) {
        std::vector<int> source_vertex_dist;
        std::vector<int> vertex_sink_dist;
        findSourceVertexDist(network, source_vertex_dist);
        findVertexSinkDist(network, vertex_sink_dist);

        while (source_vertex_dist[network.getSink()] != -1) {

            auto isEdgeOnShortestPath = [source_vertex_dist, vertex_sink_dist]
                    (const NFlow::Network::EdgeIterator &edgeIterator, NFlow::Network &network) {
                return (source_vertex_dist[edgeIterator.getStart()] + 1LL + vertex_sink_dist[edgeIterator.getFinish()]
                        == source_vertex_dist[network.getSink()]) && edgeIterator.getResidualCapacity() > 0LL;
            };

            HelperGraph helper_graph(network, isEdgeOnShortestPath);
            helper_graph.findPotentials();
            helper_graph.removalAllBlockedVertex();


            while (helper_graph.existUpdateBlockingFlow()) {
                HelperGraph::TVertex &reference_vertex = helper_graph.getReferenceNode();
                NFlow::TFlow flow = reference_vertex.getPotential();
                PushVertexSinkFlow(reference_vertex, flow, helper_graph);
                PushSourceVertexFlow(reference_vertex, flow, helper_graph);
                helper_graph.removalAllBlockedVertex();
            }

            findSourceVertexDist(network, source_vertex_dist);
            findVertexSinkDist(network, vertex_sink_dist);
        }
    }
};

#endif