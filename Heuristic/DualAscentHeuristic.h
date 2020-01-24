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
        DualAscentHeuristic(SteinerInstance *instance, unordered_map<unsigned int, unsigned int> *tmap,
                            unordered_set<unsigned int> *terminals,
                            unsigned int root) : instance_(instance), tmap_(tmap), terminals_(terminals), root_(root) {

        }

        ~DualAscentHeuristic() {
            for (const auto &elem: cache_) {
                delete[] elem.second;
            }
        }

        unsigned int calculate(unsigned int n, dynamic_bitset<> *label);

    private:
        SteinerInstance *instance_;
        unordered_map<unsigned int, unsigned int> *tmap_;
        unordered_set<unsigned int> *terminals_;
        unsigned int root_;
        unordered_map<dynamic_bitset<>, unsigned int *> cache_;
        unsigned int queryCount_ = 0;

        unsigned int *precalculate(dynamic_bitset<> *label);
    };
}

#endif //STEINER_DUALASCENTHEURISTIC_H
