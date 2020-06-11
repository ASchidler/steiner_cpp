//
// Created on 1/23/20.
//

#include "DualAscentHeuristic.h"
#include "../Algorithms/DualAscent.h"

using namespace steiner;

template <typename T>
cost_id DualAscentHeuristic<T>::calculate(node_id n, const T label, const cost_id ub) {
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

template <typename T>
cost_id* DualAscentHeuristic<T>::precalculate(const T label, const cost_id ub) {
    //TODO: The test and choice for method is missing, i.e. implement multiple methods...

    auto result = DualAscent::calculateInt<T>(instance_->getGraph(), root_, label, nTerminals_+1, nNodes_);
    result->g->findDistances(root_, ub);
    auto nodeBounds = new cost_id[instance_->getGraph()->getMaxNode()];
    for(auto i : instance_->getGraph()->getNodes()) {
        nodeBounds[i] = result->cost + result->g->getDistances()[root_][i];
    }
    cache_.emplace(label, nodeBounds);
    delete result;

    return nodeBounds;
}

template class steiner::DualAscentHeuristic<uint16_t>;
template class steiner::DualAscentHeuristic<uint32_t>;
template class steiner::DualAscentHeuristic<uint64_t>;
template class steiner::DualAscentHeuristic<uint128_type>;