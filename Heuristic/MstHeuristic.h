//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_MSTHEURISTIC_H
#define STEINER_MSTHEURISTIC_H

#include "SteinerHeuristic.h"
#include "../Graph.h"
// TODO: Put the hashing for bitsets somewhere else...
#include "../HashSetLabelStore.h"
namespace steiner {
    class MstHeuristic : public SteinerHeuristic {
    public:
        MstHeuristic(Graph* g, unordered_map<unsigned int, unsigned int>* tmap, unordered_set<unsigned int>* terminals,
                     unsigned int root) : g_(g), tmap_(tmap), terminals_(terminals), root_(root) {

        }
        unsigned int calculate(unsigned int n, dynamic_bitset<> *label) override;

    private:
        Graph* g_;
        unordered_map<unsigned int, unsigned int>* tmap_;
        unordered_set<unsigned int>* terminals_;
        unsigned int root_;
        unordered_map<dynamic_bitset<>, unsigned int> cache_;

        unsigned int calcMst(vector<unsigned int>& ts);
    };
}


#endif //STEINER_MSTHEURISTIC_H
