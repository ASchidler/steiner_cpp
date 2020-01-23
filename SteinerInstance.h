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
        SteinerInstance(Graph *g, unordered_set<unsigned int> *terminals) : g_(g) {
            for (auto t: *terminals) {
                terminals_.insert(g->getNodeMapping(t));
            }

            // Find distances from terminals to other nodes.
            for (auto t: terminals_) {
                g->findDistances(t);
            }
            // Now calculate the closest terminals
            closest_terminals_ = new Neighbor*[g->getNumNodes()];

            for(int n=0; n < g->getNumNodes(); n++) {
                auto size = terminals->size();
                if (terminals_.find(n) != terminals_.end())
                    size--;
                closest_terminals_[n] = new Neighbor[size];

                int i = 0;
                for (auto t: terminals_) {
                    if (t != n) {
                        closest_terminals_[n][i].node = t;
                        closest_terminals_[n][i].cost = g->getDistances()[t][n];
                        i++;
                    }
                }
                std::sort(closest_terminals_[n], closest_terminals_[n] + size, greater<Neighbor>());
            }
        }

        Graph *getGraph() {
            return this->g_;
        }

        unordered_set<unsigned int> *getTerminals() {
            return &(this->terminals_);
        }

        Neighbor* getClosestTerminals(unsigned int n) {
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
        unordered_set<unsigned int> terminals_;
        Neighbor** closest_terminals_;
    };
}
#endif //STEINER_STEINERINSTANCE_H
