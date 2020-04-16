//
// Created by aschidler on 1/23/20.
//

#include "DualAscentHeuristic.h"
#include "../Algorithms/DualAscent.h"

using namespace steiner;

cost_id DualAscentHeuristic::calculate(node_id n, const dynamic_bitset<> *label, const cost_id ub) {
    //Special case where only on terminal left...
    if (label->all())
        return instance_->getGraph()->getDistances()[root_][n];

    auto result = cache_.find(*label);
    cost_id* values;
    if (result != cache_.end())
        values = result->second;
    else
        values = precalculate(label, ub);


    //TODO: Add memory saver (?)
    return values[n];
}

cost_id* DualAscentHeuristic::precalculate(const dynamic_bitset<> *label, const cost_id ub) {
    //TODO: The test and choice for method is missing, i.e. implement multiple methods...

    auto result = DualAscent::calculate(instance_->getGraph(), root_, label, nTerminals_+1, nNodes_);
    result->g->findDistances(root_, ub);
    auto nodeBounds = new cost_id[instance_->getGraph()->getMaxNode()];
    for(auto i : instance_->getGraph()->getNodes()) {
        nodeBounds[i] = result->cost + result->g->getDistances()[root_][i];
    }
    cache_.emplace(*label, nodeBounds);
    delete result;

    return nodeBounds;
}