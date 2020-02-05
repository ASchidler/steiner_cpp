//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_CPP_GRAPH_H
#define STEINER_CPP_GRAPH_H
#include <cstdint>
#include <bits/stdc++.h>
#include "Steiner.h"

using namespace std;

namespace steiner {
    struct NodeWithCost {
        NodeWithCost(node_id node, cost_id cost) : node(node), cost(cost) {
        }
        NodeWithCost() : node(0), cost(0) {
        }
        node_id node;
        cost_id cost;

        // TODO: These are actually the wrong way so that priority queues are min queues...
        bool operator<(const NodeWithCost& p2) const {
            return cost > p2.cost;
        }
        bool operator>(const NodeWithCost& p2) const {
            return cost < p2.cost;
        }
    };

    struct Edge {
        Edge(node_id u, node_id v, cost_id cost) : u(u), v(v), cost(cost) {
        }

        Edge():u(0), v(0), cost(MAXCOST) {
        }

        node_id u;
        node_id v;
        cost_id cost;

        bool operator<(const Edge& p2) const {
            return cost < p2.cost;
        }

        bool operator>(const Edge& p2) const {
            return cost > p2.cost;
        }

        bool operator==(const Edge& p2) const {
            return cost == p2.cost && ((u == p2.u && v == p2.v) || (u == p2.v && v == p2.u));
        }
    };

    struct ContractedEdge {
        ContractedEdge(node_id removed, node_id target, node_id n, cost_id c) :
            removed(removed), target(target), n(n), c(c)
        {
        }
        node_id removed;
        node_id target;
        node_id n;
        cost_id c;
    };


    class Graph {
    public:
        ~Graph() {
            discardDistances();
        }
        Graph() = default;

        explicit Graph(node_id nNodes) {
            for(node_id i=0; i < nNodes; i++) {
                nb.emplace_back();
            }
        }

        Graph(Graph& g, bool copyMapping) : nb(vector<unordered_map<node_id, cost_id>>(g.nb)), nodes_(g.nodes_) {
            if (copyMapping) {
                nodeMap_ = g.nodeMap_;
                nodeReverseMap_ = g.nodeReverseMap_;
            }
        }

        class EdgeIterator {
        public:
            explicit EdgeIterator(unordered_set<node_id>* nodes, vector<unordered_map<node_id, cost_id>>* nb) : nodes_(nodes), nb_(nb){
                nodeState = nodes->begin();
                nbState = (*nb)[*nodeState].begin();
                findNext();
            }

            Edge operator*() {
                return {*nodeState, nbState->first, nbState->second};
            }

            EdgeIterator &operator++() {
                nbState++;
                findNext();
                return *this;
            }

            bool hasElement() {
                return nodeState != nodes_->end();
            }

        private:
            friend class Graph;
            unordered_set<node_id>::iterator nodeState;
            unordered_map<node_id, cost_id>::iterator nbState;
            unordered_set<node_id>* nodes_;
            vector<unordered_map<node_id, cost_id>>* nb_;
            void findNext() {
                while(nodeState != nodes_->end()) {
                    while(nbState != (*nb_)[*nodeState].end()) {
                        if (*nodeState < nbState->first)
                            return;
                        nbState++;
                    }
                    nodeState++;
                    if (nodeState != nodes_->end())
                        nbState = (*nb_)[*nodeState].begin();
                }
            }
        }; //EdgeIterator

        node_id addNode(node_id u);
        bool addMappedEdge(node_id u, node_id v, cost_id cost);
        bool addEdge(node_id u, node_id v, cost_id cost);

        node_id getMaxNode() {
            return this->nb.size();
        }

        node_id getNumNodes() {
            return this->nodes_.size();
        }

        // TODO: Return reference!
        unordered_set<node_id>& getNodes(){
            return nodes_;
        }

        vector<unordered_map<node_id, cost_id>> nb;

        node_id getNodeMapping(node_id externalId);
        node_id getReverseMapping(node_id internal);

        void findDistances(node_id u);
        cost_id** getDistances() {
            return distances_;
        }
        void discardDistances();
        vector<node_id> findPath(node_id u, node_id v);

        bool shrink();

        unordered_set<node_id>::iterator removeNode(node_id u);
        unordered_set<node_id>::iterator removeNode(unordered_set<node_id>::iterator u);
        void removeEdge(node_id u, node_id v);
        EdgeIterator removeEdge(EdgeIterator);
        // TODO: This is really ugly (result)
        unordered_set<node_id>::iterator contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result);
        EdgeIterator findEdges() {
            return EdgeIterator(&nodes_, &nb);
        }
        void switchVertices(node_id n1, node_id n2);
        bool checkConnectedness(node_id nodeLimit, bool clean);
        Graph* mst();
        cost_id mst_sum();
        cost_id getCost() {
            cost_id sum = 0;
            for(auto n: nodes_) {
                for(auto& b: nb[n]){
                    if (n < b.first)
                        sum += b.second;
                }
            }
            return sum;
        }
    private:
        unordered_set<node_id> nodes_;
        unordered_map<node_id, node_id> nodeMap_;
        unordered_map<node_id, node_id> nodeReverseMap_;
        cost_id** distances_ = nullptr;
    };

    struct HeuristicResult {
        HeuristicResult(cost_id bound, Graph* g, node_id root) :
                bound(bound), g(g), root(root)
        {}
        ~HeuristicResult() {
            delete g;
        }

        cost_id bound;
        Graph* g;
        node_id root;
    };
}

namespace std {
    template <>
    struct hash<steiner::Edge>
    {
        size_t operator()(const steiner::Edge& k) const
        {
            return hash<node_id>()(k.u) ^ hash<node_id>()(k.v);
        }
    };
}


#endif //STEINER_CPP_GRAPH_H
