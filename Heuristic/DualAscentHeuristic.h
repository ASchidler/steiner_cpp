//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_DUALASCENTHEURISTIC_H
#define STEINER_DUALASCENTHEURISTIC_H
#include "../SteinerInstance.h"
#include "SteinerHeuristic.h"
#include "../SteinerInstance.h"
#include "../HashSetLabelStore.h"

using namespace steiner;
using namespace boost;

namespace steiner {
    class DualAscentHeuristic : public SteinerHeuristic {
    public:
        DualAscentHeuristic(SteinerInstance* instance, unordered_map<node_id, node_id>* tmap, unordered_set<node_id>* terminals,
                            node_id root) : instance_(instance), tmap_(tmap), terminals_(terminals), root_(root) {

        }

        ~DualAscentHeuristic() {
            for (const auto &elem: cache_) {
                delete[] elem.second;
            }
        }

        cost_id calculate(node_id n, dynamic_bitset<> *label);

    private:
        SteinerInstance* instance_;
        unordered_map<node_id, node_id>* tmap_;
        unordered_set<node_id>* terminals_;
        node_id root_;
        unordered_map<dynamic_bitset<>, cost_id*> cache_;
        unsigned int queryCount_ = 0;

        cost_id *precalculate(dynamic_bitset<> *label);
    };
}

#endif //STEINER_DUALASCENTHEURISTIC_H
