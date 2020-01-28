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
            return cost > p2.cost || (cost == p2.cost && node > p2.node);
        }
        bool operator>(const NodeWithCost& p2) const {
            return cost < p2.cost || (cost == p2.cost && node > p2.node);
        }
    };

    struct Edge {
        Edge(node_id u, node_id v, cost_id cost) : u(u), v(v), cost(cost) {
        }

        node_id u;
        node_id v;
        cost_id cost;
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
            if (distances_ != nullptr) {
                for (size_t i = 0; i < sizeof(distances_); i++) {
                    delete[] distances_[i];
                }
                delete[] distances_;
            }
        }
        node_id addNode(node_id u);
        bool addMappedEdge(node_id u, node_id v, cost_id cost);
        bool addEdge(node_id u, node_id v, cost_id cost);

        node_id getMaxNode() {
            return this->nb.size();
        }

        node_id getNumNodes() {
            return this->nodes_.size();
        }

        unordered_set<node_id>* getNodes(){
            return &nodes_;
        }

        vector<unordered_map<node_id, cost_id>> nb;

        node_id getNodeMapping(node_id externalId);
        node_id getReverseMapping(node_id internal);

        void findDistances(node_id u);
        cost_id** getDistances() {
            return distances_;
        }

        void removeNode(node_id u);
        unordered_set<node_id>::iterator removeNode(unordered_set<node_id>::iterator u);
        void removeEdge(node_id u, node_id v);
        // TODO: This is really ugly (result)
        void contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result);
        void switchVertices(node_id n1, node_id n2);
        bool isConnected();
        Graph* copy(bool copyMapping);
    private:
        unordered_set<node_id> nodes_;
        unordered_map<node_id, node_id> nodeMap_;
        unordered_map<node_id, node_id> nodeReverseMap_;
        cost_id** distances_ = nullptr;
    };
}
#endif //STEINER_CPP_GRAPH_H


