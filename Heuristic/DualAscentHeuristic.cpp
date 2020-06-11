//
// Created on 1/23/20.
//

#include "DualAscentHeuristic.h"
#include "../Algorithms/DualAscent.h"

using namespace steiner;

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
cost_id DualAscentHeuristic<T, T2>::calculate(node_id n, const T label, const cost_id ub) {
    //Special case where only on terminal left...
    if (label == maxTerminal_)
        return instance_->getGraph()->getDistances()[root_][n];

    auto result = cache_.find(label);
    cost_id* values;
    if (result != cache_.end())
        values = result->second;
    else
        values = precalculate(label, ub);


    //TODO: Add memory saver (?)
    return values[n];
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
cost_id* DualAscentHeuristic<T, T2>::precalculate(const T label, const cost_id ub) {
    //TODO: The test and choice for method is missing, i.e. implement multiple methods...

    auto result = DualAscent::calculate<T>(instance_->getGraph(), root_, label, nTerminals_+1, nNodes_);
    result->g->findDistances(root_, ub);
    auto nodeBounds = new cost_id[instance_->getGraph()->getMaxNode()];
    for(auto i : instance_->getGraph()->getNodes()) {
        nodeBounds[i] = result->cost + result->g->getDistances()[root_][i];
    }
    cache_.emplace(label, nodeBounds);
    delete result;

    return nodeBounds;
}