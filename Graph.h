//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_CPP_GRAPH_H
#define STEINER_CPP_GRAPH_H
#include <cstdint>
#include <bits/stdc++.h>
#include "steiner.h"

using namespace std;

//TODO: Make directed possible? Make a switch when creating the graph...
//TODO: Use nNodes in constructor as upper bound, count nodes when added, and map them? Coordinate with instance type

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
        void addEdge(node_id u, node_id v, cost_id cost);

        node_id getNumNodes() {
            return this->nodes_.size();
        }

        vector<node_id>* getNodes(){
            return &nodes_;
        }

        vector<unordered_map<node_id, cost_id>> nb;

        node_id getNodeMapping(node_id externalId);

        void findDistances();
        void findDistances(node_id u);
        cost_id** getDistances() {
            return distances_;
        }

        Graph* copy();
    private:
        vector<node_id> nodes_;
        unordered_map<node_id, node_id> nodeMap_;
        unordered_map<node_id, node_id> nodeReverseMap_;
        cost_id** distances_ = nullptr;
    };
}
#endif //STEINER_CPP_GRAPH_H


