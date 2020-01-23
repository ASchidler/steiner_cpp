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
        }

        Graph *getGraph() {
            return this->g_;
        }

        unordered_set<unsigned int> *getTerminals() {
            return &(this->terminals_);
        }

    private:
        Graph *g_;
        unordered_set<unsigned int> terminals_;
    };
}
#endif //STEINER_STEINERINSTANCE_H
