//
// Created by andre on 22.03.20.
//

#include "GraphPruner.h"

bool GraphPruner::prune() {
    auto g = instance_.getGraph();

    auto instanceVor = VoronoiDiagram::create(g, instance_.getNumTerminals());
    auto radius = instanceVor->getRadiusSum(instance_.getNumTerminals());

    // Find upper bounds, such that a constant fraction of vertices will be removed
    cost_id values[g->getNumNodes()];
    cost_id* cVal = values;
    for(auto n: g->getNodes()) {
        *cVal = instanceVor->closest[n].cost + instanceVor->second[n].cost + radius;
        cVal++;
    }
    std::sort(values, cVal, std::greater<>());
    cost_id cBound = values[g->getNumNodes() / 7];

    // Remove the nodes.
    for(auto it = g->getNodes().begin(); it != g->getNodes().end(); ) {
        if (*it >= instance_.getNumTerminals()  && lastResult_->g->getNodes().find(*it) == lastResult_->g->getNodes().end()) {
            auto lb = instanceVor->closest[*it].cost + instanceVor->second[*it].cost + radius;
            // Try not to remove nodes from the best solution to maintain connectedness.
            // Due to the reductions this is not a guarantee
            if (lb > cBound) {
                it = g->removeNode(it);
                continue; // Avoid incrementing of iterator
            }
        }
        it++;
    } // End remove nodes

    instance_.setSteinerDistanceState(SteinerInstance::invalid);
    instance_.setDistanceState(SteinerInstance::invalid);
    instance_.setApproximationState(SteinerInstance::invalid);

    return instance_.getGraph()->checkConnectedness(instance_.getNumTerminals(), true);
}

std::shared_ptr<SteinerResult> GraphPruner::approximate(bool unreduce) {
    // TODO: Maybe incorporate non-terminals here?
    auto rt = random() % instance_.getNumTerminals();
    auto result = ShortestPath::calculate(rt, *instance_.getGraph(), instance_.getNumTerminals());

    if (unreduce) {
        result->g->remap(*instance_.getGraph());
        reducer_.reset();
        reducer_.unreduce(result.get());

        if (lastResult_ == nullptr || lastResult_->cost > result->cost)
            lastResult_ = result;
    }

    return result;
}

void GraphPruner::reduce() {
    reducer_.reduce();
}

void GraphPruner::unreduce(const std::shared_ptr<SteinerResult>& result) {
    result->g->remap(*instance_.getGraph());
    reducer_.reset();
    reducer_.unreduce(result.get());

    if (lastResult_ == nullptr || lastResult_->cost > result->cost)
        lastResult_ = result;
}
