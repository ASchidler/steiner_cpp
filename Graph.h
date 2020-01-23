//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_CPP_GRAPH_H
#define STEINER_CPP_GRAPH_H
#include <cstdint>
#include <bits/stdc++.h>

using namespace std;

//TODO: Make directed possible? Make a switch when creating the graph...
//TODO: Use nNodes in constructor as upper bound, count nodes when added, and map them? Coordinate with instance type

namespace steiner {
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
        unsigned int addVertex(unsigned int u);
        void addEdge(unsigned int u, unsigned int v, unsigned int cost);

        unsigned int getNumNodes() {
            return this->nodes_.size();
        }

        struct Neighbor {
            Neighbor(unsigned int node, unsigned int cost) : node(node), cost(cost) {
            }
            unsigned int node;
            unsigned int cost;

            bool operator<(const Neighbor& p2) const {
                return cost > p2.cost || (cost == p2.cost && node > p2.node);
            }
        };

        vector<vector<Neighbor>> nb;

        unsigned int getNodeMapping(unsigned int externalId);

        // TODO: Restricted Version to only some nodes?
        void findDistances();
        void findDistances(unsigned int u);
        unsigned int** getDistances() {
            return distances_;
        }
    private:
        vector<unsigned int> nodes_ = vector<unsigned int>();
        unordered_map<unsigned int, unsigned int> nodeMap_ = unordered_map<unsigned int, unsigned int>();
        unsigned int maxNodeId_ = 0;
        unsigned int** distances_ = nullptr;
    };
}
#endif //STEINER_CPP_GRAPH_H

