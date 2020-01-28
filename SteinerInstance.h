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
        SteinerInstance(Graph *g, vector<node_id> *terminals);
        ~SteinerInstance() {
            for(int n=0; n < g_->getMaxNode(); n++) {
                delete[] closest_terminals_[n];
            }
            delete[] closest_terminals_;
        }

        node_id getNumTerminals() {
            return nTerminals;
        }

        bool addEdge(node_id u, node_id v, cost_id c);
        unordered_set<node_id>::iterator removeNode(node_id u);
        unordered_set<node_id>::iterator removeNode(unordered_set<node_id>::iterator u);
        void removeEdge(node_id u, node_id v);
        unordered_set<node_id>::iterator contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result);
        NodeWithCost* getClosestTerminals(node_id n);
        enum ValueState { lower, exact, higher };

        Graph *getGraph() {
            return this->g_;
        }
    private:
        Graph *g_;
        NodeWithCost** closest_terminals_ = nullptr;
        node_id nTerminals = 0;
        inline node_id removeNode_(node_id u);
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
