//
// Created on 1/22/20.
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
            return cost > p2.cost || cost == p2.cost && node > p2.node;
        }
        bool operator>(const NodeWithCost& p2) const {
            return cost < p2.cost || cost == p2.cost && node < p2.node;
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
            nb.resize(nNodes);
        }

        Graph(Graph& g, bool copyMapping) : nb(vector<map<node_id, cost_id, NodeIdHash>>(g.nb)), nodes_(g.nodes_) {
            if (copyMapping) {
                nodeMap_ = g.nodeMap_;
                nodeReverseMap_ = g.nodeReverseMap_;
            }
        }

        class EdgeIterator {
        public:
            explicit EdgeIterator(set<node_id>* nodes, vector<map<node_id, cost_id, NodeIdHash>>* nb) : nodes_(nodes), nb_(nb){
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
            set<node_id>::iterator nodeState;
            map<node_id, cost_id, NodeIdHash>::iterator nbState;
            set<node_id>* nodes_;
            vector<map<node_id, cost_id, NodeIdHash>>* nb_;
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

        node_id addMappedNode(node_id u);
        void addUnmappedNode(node_id u) {
            if (nodes_.insert(u).second) {
                nodeMap_[u] = u;
                nodeReverseMap_[u] = u;

                if (u >= nb.size()) {
                    nb.resize(u+1);
                }
            }
        }
        bool addMappedEdge(node_id u, node_id v, cost_id cost);
        bool addEdge(node_id u, node_id v, cost_id cost);

        void remap(Graph& g);

        node_id getMaxNode() const {
            return this->nb.size();
        }

        node_id getNumNodes() {
            return this->nodes_.size();
        }

        set<node_id>& getNodes(){
            return nodes_;
        }

        vector<map<node_id, cost_id, NodeIdHash>> nb;

        node_id getNodeMapping(node_id externalId);
        node_id getReverseMapping(node_id internal);

        void findDistances(node_id u, cost_id ub);
        cost_id** getDistances() {
            return distances_;
        }
        bool hasDistances() {
            return distances_ != nullptr;
        }
        void discardDistances();
        vector<node_id> findPath(node_id u, node_id v);

        bool shrink();

        set<node_id>::iterator removeNode(node_id u);
        set<node_id>::iterator removeNode(set<node_id>::iterator u);
        bool adaptWeight(node_id up, node_id vp, cost_id original, cost_id modified);
        void removeEdge(node_id u, node_id v);
        EdgeIterator removeEdge(EdgeIterator);
        // TODO: This is really ugly (result)
        set<node_id>::iterator contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result);
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
        cost_id getMaxKnownDistance() const {
            // The idea here is that in the first run, the value is 0, therefore it will cause a bucket queue
            // In subsequent calls, the maximum known distance is known and will work as a kind of upper bound
            // It may be beneficial to reset this, in case reductions lower this significantly.
            return maxKnownDistance_;
        }

        node_id getOriginalNumEdges() const {
            return originalNumEdges_;
        }

        node_id getNumEdges() {
            if (changed_) {
                num_edges_ = 0;
                for (auto &cN: nodes_) {
                    for (auto &cE: nb[cN]) {
                        if (cN < cE.first)
                            num_edges_++;
                    }
                }
            }

            return num_edges_;
        }

    private:
        set<node_id> nodes_;
        unordered_map<node_id, node_id> nodeMap_;
        unordered_map<node_id, node_id> nodeReverseMap_;
        cost_id** distances_ = nullptr;
        node_id distanceInit_ = 0;
        cost_id maxKnownDistance_ = 0;
        node_id originalNumEdges_ = 0;
        node_id num_edges_ = 0;
        bool changed_ = true;
    };

    struct SteinerResult {
        SteinerResult(cost_id bound, Graph* g, node_id root) :
                cost(bound), g(g), root(root)
        {}
        ~SteinerResult() {
            delete g;
        }

        cost_id cost;
        Graph* g;
        node_id root;

        bool operator<(const SteinerResult& p2) const {
            return cost > p2.cost;
        }
        bool operator>(const SteinerResult& p2) const {
            return cost < p2.cost;
        }
        static bool cmp(SteinerResult* r1, SteinerResult* r2) {
            return r1->cost < r2->cost || (r1->cost == r2->cost && r1->root < r2->root);
        }
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
