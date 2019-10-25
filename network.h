#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <vector>
#include <map>
#include <stdexcept>

namespace NFlow {
    namespace NInner {
        typedef long long TFlow;
        typedef unsigned int TVertex;
        typedef unsigned int TVertexNumber;

        struct SourceIsEqualToSinkException : public std::logic_error {
            explicit SourceIsEqualToSinkException() : std::logic_error("Source is equal to sink") {}
        };

        struct TooBigSourceException : public std::out_of_range {
            explicit TooBigSourceException() : std::out_of_range("Too big source or sink") {}
        };

        struct TooSmallSourceException : public std::out_of_range {
            explicit TooSmallSourceException() : std::out_of_range("Too small source or sink") {}
        };

        struct NegativeCapacityException : public std::invalid_argument {
            explicit NegativeCapacityException() : std::invalid_argument("Negative capacity") {}
        };

        struct InvalidEdgeIterator : public std::runtime_error {
            explicit InvalidEdgeIterator() : std::runtime_error("Taking a value from an invalid iterator") {}
        };

        struct Edge {
            TVertex start, finish;
            TFlow capacity, flow;
        };

        class Network {
        public:
            Network(TVertexNumber n, TVertex s, TVertex t) :lasts_(n, -1), vertexNumber_(n), source_(s), sink_(t) {
                if (s >= n || t >= n)
                    throw TooBigSourceException();
                else if (s < static_cast<TVertex>(0) || t < static_cast<TVertex>(0)) {
                    throw TooSmallSourceException();
                }
                else if (s == t)
                    throw SourceIsEqualToSinkException();
            }

            [[nodiscard]] TVertex getSource() const {
                return source_;
            }

            [[nodiscard]] TVertex getSink() const {
                return sink_;
            }

            [[nodiscard]] TVertexNumber getVertexNumber() const {
                return vertexNumber_;
            }

            void addEdge(TVertex start, TVertex finish, TFlow capacity) {
                if (capacity < static_cast<TFlow>(0))
                    throw NegativeCapacityException();
                addEdgeLocal_(start, finish, capacity);
                addEdgeLocal_(finish, start, static_cast<TFlow>(0));
            }

            void deleteAllEdges() {
                lasts_.assign(getVertexNumber(), -1);
                edges_.clear();
                uk_.clear();
            }

            class EdgeIterator {
            public:
                friend class Network;

                int edgeID_;
                Network& myNetwork_;

                EdgeIterator(int id, Network& net) :edgeID_(id), myNetwork_(net) {}

            public:

                EdgeIterator reverse_edge() {
                    EdgeIterator a(*this);
                    if (a.edgeID_ % 2 == 0) {
                        ++a.edgeID_;
                    } else {
                        --a.edgeID_;
                    }
                    return  a;
                }

                EdgeIterator& operator=(const EdgeIterator& val) {
                    edgeID_ = val.edgeID_;
                    return (*this);
                }

                [[nodiscard]] bool isValid() const {
                    return edgeID_ >= 0;
                }

                EdgeIterator& goNext() {
                    if (!isValid())
                        throw InvalidEdgeIterator();
                    edgeID_ = myNetwork_.uk_[edgeID_];
                    return *this;
                }

                [[nodiscard]] TVertex getStart() const {
                    return myNetwork_.edges_[edgeID_].start;
                }

                [[nodiscard]] TVertex getFinish() const {
                    return myNetwork_.edges_[edgeID_].finish;
                }

                [[nodiscard]] TFlow getCapacity() const {
                    return myNetwork_.edges_[edgeID_].capacity;
                }

                [[nodiscard]] TFlow getFlow() const {
                    return myNetwork_.edges_[edgeID_].flow;
                };

                [[nodiscard]] TFlow getResidualCapacity() const {
                    return getCapacity() - getFlow();
                }

                void pushFlow(TFlow flow) const {
                    myNetwork_.pushFlow_(edgeID_, flow);
                }

            };

            EdgeIterator getListBegin(TVertex v) {
                return EdgeIterator(lasts_[v], *this);
            }

        private:
            std::vector<Edge> edges_;
            std::vector<int> lasts_, uk_;
            TVertexNumber vertexNumber_;
            TVertex source_, sink_;

            void addEdgeLocal_(TVertex start, TVertex finish, TFlow capacity) {
                edges_.emplace_back(Edge{start, finish, capacity, static_cast<TFlow>(0)});
                uk_.push_back(lasts_[start]);
                lasts_[start] = edges_.size() - 1;
            }

            void pushFlow_(size_t edge, TFlow flow) {
                edges_[edge].flow += flow;
                edges_[edge ^ 1u].flow -= flow;
            }

        };

    }

    using NInner::Network, NInner:: TFlow, NInner::TVertex;

    TFlow getFlowValue(Network &network) {
        TFlow ans(0);
        for (Network::EdgeIterator it = network.getListBegin(network.getSource()); it.isValid(); it.goNext())
            ans += it.getFlow();
        return ans;
    }

    void RemoveAllEdgesDuplicates(Network& network) {
        std::map<std::pair<NFlow::TVertex, NFlow::TVertex>, NFlow::TFlow> edge_capacity;

        for (size_t v = 0; v < network.getVertexNumber(); ++v) {
            for (auto it = network.getListBegin(v); it.isValid(); it.goNext()) {
                if (it.getCapacity() > 0LL) {
                    edge_capacity[{it.getStart(), it.getFinish()}] += it.getCapacity();
                }
            }
        }

        network.deleteAllEdges();
        for (const auto& [edge, capacity] : edge_capacity) {
            network.addEdge(edge.first, edge.second, capacity);
        }
    }
}

#endif