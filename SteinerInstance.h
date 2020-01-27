//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_STEINERINSTANCE_H
#define STEINER_STEINERINSTANCE_H
#include "Graph.h"
#include <bits/stdc++.h>

#include <utility>

using namespace std;

namespace steiner {
    class SteinerInstance {
    public:
        SteinerInstance(Graph *g, unordered_set<node_id> *terminals);

        Graph *getGraph() {
            return this->g_;
        }

        unordered_set<node_id> *getTerminals() {
            return &(this->terminals_);
        }

        ~SteinerInstance() {
            for(int n=0; n < g_->getNumNodes(); n++) {
                delete[] closest_terminals_[n];
            }
            delete[] closest_terminals_;
        }

        bool addEdge(node_id u, node_id v, cost_id c);
        void removeNode(node_id u);
        void removeEdge(node_id u, node_id v);
        void contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result);
        NodeWithCost* getClosestTerminals(node_id n);
    private:
        Graph *g_;
        unordered_set<node_id> terminals_;
        NodeWithCost** closest_terminals_ = nullptr;


    };

    struct MergedEdges {
        MergedEdges(node_id removed, node_id u, node_id v, cost_id cu, cost_id cv) :
            newEdge(u, v, cu + cv),
            oldEdge1(removed, u, cu),
            oldEdge2(removed, v, cv) {

        }
        Edge newEdge;
        Edge oldEdge1;
        Edge oldEdge2;
    };
}
#endif //STEINER_STEINERINSTANCE_H
