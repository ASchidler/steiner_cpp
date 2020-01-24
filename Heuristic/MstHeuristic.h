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
        MstHeuristic(SteinerInstance* instance, unordered_map<node_id, node_id>* tmap, unordered_set<node_id>* terminals,
                     node_id root) : instance_(instance), tmap_(tmap), terminals_(terminals), root_(root) {

        }
        cost_id calculate(node_id n, dynamic_bitset<> *label) override;

    private:
        SteinerInstance* instance_;
        unordered_map<node_id, node_id>* tmap_;
        unordered_set<node_id>* terminals_;
        node_id root_;
        unordered_map<dynamic_bitset<>, cost_id> cache_;

        cost_id calcMst(dynamic_bitset<>* label);
    };
}


#endif //STEINER_MSTHEURISTIC_H
