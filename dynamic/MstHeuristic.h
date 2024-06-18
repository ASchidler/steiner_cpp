//
// Created by aschidler on 1/23/20.
//

#ifndef DYNAMIC_STEINER_MSTHEURISTIC_H
#define DYNAMIC_STEINER_MSTHEURISTIC_H

#include "SteinerHeuristic.h"
#include "../Graph.h"
#include "../SteinerInstance.h"
// TODO: Maybe implement the STP heuristic computing STP for 2-3 terminals?
#include "../HashSetLabelStore.h"

using std::unordered_map;

namespace steiner {
    class DynamicMstHeuristic : public DynamicSteinerHeuristic {
    public:
        DynamicMstHeuristic(SteinerInstance* instance, node_id root, node_id nTerminals)
                : instance_(instance), root_(root), nTerminals_(nTerminals) {

        }
        cost_id calculate(node_id n, const dynamic_bitset<> *label, cost_id ub) override;

    private:
        SteinerInstance* instance_;
        node_id root_;
        unordered_map<dynamic_bitset<>, cost_id> cache_;
        node_id nTerminals_;
        cost_id calcMst(const dynamic_bitset<>* label);
    };
}


#endif //DYNAMIC_STEINER_MSTHEURISTIC_H
