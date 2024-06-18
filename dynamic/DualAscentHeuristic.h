//
// Created by aschidler on 1/23/20.
//

#ifndef DYNAMIC_STEINER_DUALASCENTHEURISTIC_H
#define DYNAMIC_STEINER_DUALASCENTHEURISTIC_H
#include "../SteinerInstance.h"
#include "SteinerHeuristic.h"
#include "../SteinerInstance.h"
#include "../HashSetLabelStore.h"

using namespace steiner;

namespace steiner {
    class DynamicDualAscentHeuristic : public DynamicSteinerHeuristic {
    public:
        DynamicDualAscentHeuristic(SteinerInstance* instance, node_id root, node_id nTerminals, node_id numNodes)
        : instance_(instance), root_(root), nTerminals_(nTerminals), nNodes_(numNodes) {

        }

        ~DynamicDualAscentHeuristic() override {
            for (const auto &elem: cache_) {
                delete[] elem.second;
            }
        }

        cost_id calculate(node_id n, const dynamic_bitset<> *label, cost_id ub) override;

    private:
        SteinerInstance* instance_;
        node_id root_;
        unordered_map<dynamic_bitset<>, cost_id*> cache_;
        node_id nTerminals_;
        node_id nNodes_;

        cost_id *precalculate(const dynamic_bitset<> *label, cost_id ub);
    };
}

#endif //DYNAMIC_STEINER_DUALASCENTHEURISTIC_H
