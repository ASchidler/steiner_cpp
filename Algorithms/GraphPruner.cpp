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
    cost_id cBound = values[g->getNumNodes() / 10];

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

std::shared_ptr<SteinerResult> GraphPruner::approximate() {
    // TODO: Maybe incorporate non-terminals here?
    auto rt = random() % instance_.getNumTerminals();
    auto result = ShortestPath::calculate(rt, *instance_.getGraph(), instance_.getNumTerminals());
    result->g->remap(*instance_.getGraph());
    reducer_.reset();
    reducer_.unreduce(result.get());

    if (lastResult_ == nullptr || lastResult_->cost > result->cost)
        lastResult_ = result;

    return result;
}

void GraphPruner::reduce() {
    reducer_.reduce();
}


/*
 * // Create a graph consisting of the edges in the dual ascent graph, we know that this is connected
    Graph g = Graph();
    auto edgeIt = r->g->findEdges();
    while(edgeIt.hasElement()) {
        auto e = *edgeIt;
        // findEdges returns undirected edges, so look in both directions
        if (r->g->nb[e.u][e.v] == 0 || r->g->nb[e.v][e.u] == 0)
            g.addEdge(e.u, e.v, instance->getGraph()->nb[e.u][e.v]);
        ++edgeIt;
    }

    // Use a limited reducer for this new sub-graph
    SteinerInstance s(&g, instance->getNumTerminals());
    auto red = Reducer::getMinimalReducer(&s);

    // Now periodically lower the upper bound and apply bound based reduction
    unsigned int cnt = 0;
    do {
        red.reduce();

        // Get upper bound
        //TODO: Make number of roots configurable?
        for(auto t=0; t < s.getNumTerminals() && t < 5; t++) {
            auto rt = random() % s.getNumTerminals();
            auto result = ShortestPath::calculate(rt, g, s.getNumTerminals());
            result->g->remap(g);
            red.reset();
            red.unreduce(result);
            instance->getApproximation().addToPool(result);
        }

        // TODO: Make number of pruning cycles configurable
        // TODO: Make number of prunings configurable
        // TODO: Maybe remove shortlinks from minimal reducer, voronoi calculation is expensive, we do it anyway for the reduction, can we share this?
        // Reduce, use simpler bound reduction based on voronoi regions
        if (cnt <= 3) {
            // Find Voronoi, required for reduction
            auto instanceVor = voronoi(&g, s.getNumTerminals());
            auto radius = instanceVor->getRadiusSum(s.getNumTerminals());
            auto cBest = instance->getApproximation().getBest();

            // Find upper bounds, such that a constant fraction of vertices will be removed
            cost_id values[g.getNumNodes()];
            cost_id* cVal = values;
            for(auto n: g.getNodes()) {
                *cVal = instanceVor->closest[n].cost + instanceVor->second[n].cost + radius;
                cVal++;
            }
            std::sort(values, cVal, std::greater<>());
            cost_id cBound = values[g.getNumNodes() / 10];

            // Remove the nodes.
            for(auto it = g.getNodes().begin(); it != g.getNodes().end(); ) {
                if (*it >= s.getNumTerminals()) {
                    auto lb = instanceVor->closest[*it].cost + instanceVor->second[*it].cost + radius;
                    // Try not to remove nodes from the best solution to maintain connectedness.
                    // Due to the reductions this is not a guarantee
                    if (lb > cBound && cBest->g->getNodes().find(*it) == cBest->g->getNodes().end()) {
                        it = g.removeNode(it);
                        continue; // Avoid incrementing of iterator
                    }
                }
                it++;
            } // End remove nodes

            s.setSteinerDistanceState(SteinerInstance::invalid);
            s.setDistanceState(SteinerInstance::invalid);
            s.setApproximationState(SteinerInstance::invalid);
        }
        cnt++;
    // Do this several times or until the graph is disconnected
    } while(cnt < 2 && g.checkConnectedness(s.getNumTerminals(), true) && s.getNumTerminals() > 3);
 */