//
// Created by aschidler on 1/23/20.
//

#include "DualAscentHeuristic.h"
#include "../Algorithms/DualAscent.h"

using namespace steiner;

cost_id DualAscentHeuristic::calculate(node_id n, dynamic_bitset<> *label) {
    //Special case where only on terminal left...
    if (label->count() == terminals_->size())
        return instance_->getGraph()->getDistances()[root_][n];

    auto result = cache_.find(*label);
    cost_id* values;
    if (result != cache_.end())
        values = result->second;
    else
        values = precalculate(label);


    //TODO: Add memory saver
    return values[n];
}

cost_id* DualAscentHeuristic::precalculate(dynamic_bitset<> *label) {
    //TODO: The test and choice for method is missing

    unordered_set<node_id> ts;
    ts.insert(root_);
    for (auto t: *terminals_) {
        if (! label->test(((*tmap_)[t]))) {
            ts.insert(t);
        }
    }

    auto result = DualAscent::calculate(instance_->getGraph(), root_, &ts);
    result->g->findDistances(root_);
    auto nodeBounds = new cost_id[instance_->getGraph()->getNumNodes()];
    for(int i=0; i < instance_->getGraph()->getNumNodes(); i++) {
        nodeBounds[i] = result->bound + result->g->getDistances()[root_][i];
    }
    cache_.insert(pair<dynamic_bitset<>, cost_id*>(*label, nodeBounds));
    delete result;

    return nodeBounds;
}