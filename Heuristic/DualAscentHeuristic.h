//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_DUALASCENTHEURISTIC_H
#define STEINER_DUALASCENTHEURISTIC_H
#include "../SteinerInstance.h"

using namespace steiner;

class DualAscentHeuristic {
    DualAscentHeuristic(SteinerInstance* instance, unordered_map<unsigned int, unsigned int>* tmap, unordered_set<unsigned int>* terminals,
    unsigned int root) : instance_(instance), tmap_(tmap), terminals_(terminals), root_(root) {

    }

private:
    SteinerInstance* instance_;
    unordered_map<unsigned int, unsigned int>* tmap_;
    unordered_set<unsigned int>* terminals_;
    unsigned int root_;
};


#endif //STEINER_DUALASCENTHEURISTIC_H
