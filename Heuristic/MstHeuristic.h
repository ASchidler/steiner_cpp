//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_MSTHEURISTIC_H
#define STEINER_MSTHEURISTIC_H

#include "SteinerHeuristic.h"
#include "../Graph.h"
#include "../SteinerInstance.h"
// TODO: Put the hashing for bitsets somewhere else...
#include "../HashSetLabelStore.h"
namespace steiner {
    class MstHeuristic : public SteinerHeuristic {
    public:
        MstHeuristic(SteinerInstance* instance, node_id root, node_id nTerminals)
                : instance_(instance), root_(root), nTerminals_(nTerminals) {

        }
        cost_id calculate(node_id n, const dynamic_bitset<> *label) override;

    private:
        SteinerInstance* instance_;
        node_id root_;
        unordered_map<dynamic_bitset<>, cost_id> cache_;
        node_id nTerminals_;
        cost_id calcMst(const dynamic_bitset<>* label);
    };
}


#endif //STEINER_MSTHEURISTIC_H
