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
        SteinerInstance(Graph *g, unordered_set<node_id> *terminals) : g_(g) {
            for (auto t: *terminals) {
                terminals_.insert(g->getNodeMapping(t));
            }

            // Find distances from terminals to other nodes.
            for (auto t: terminals_) {
                g->findDistances(t);
            }
            // Now calculate the closest terminals
            closest_terminals_ = new NodeWithCost*[g->getNumNodes()];

            for(int n=0; n < g->getNumNodes(); n++) {
                closest_terminals_[n] = new NodeWithCost[terminals->size()];

                int i = 0;
                for (auto t: terminals_) {
                    closest_terminals_[n][i].node = t;
                    closest_terminals_[n][i].cost = g->getDistances()[t][n];
                    i++;
                }
                std::sort(closest_terminals_[n], closest_terminals_[n] + terminals->size(), greater<NodeWithCost>());
            }
        }

        Graph *getGraph() {
            return this->g_;
        }

        unordered_set<node_id> *getTerminals() {
            return &(this->terminals_);
        }

        NodeWithCost* getClosestTerminals(node_id n) {
            return closest_terminals_[n];
        }

        ~SteinerInstance() {
            for(int n=0; n < g_->getNumNodes(); n++) {
                delete[] closest_terminals_[n];
            }
            delete[] closest_terminals_;
        }
    private:
        Graph *g_;
        unordered_set<node_id> terminals_;
        NodeWithCost** closest_terminals_;
    };
}
#endif //STEINER_STEINERINSTANCE_H
