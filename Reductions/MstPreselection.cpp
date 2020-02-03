//
// Created by aschidler on 1/30/20.
//

#include "MstPreselection.h"
#include "../Algorithms/UnionFind.h"

node_id steiner::MstPreselection::reduce(node_id currCount, node_id prevCount) {
    if (instance->getNumTerminals() <= 2)
        return 0;

    instance->requestDistanceState(SteinerInstance::higher);

    vector<Edge> edges;
    UnionFind components = UnionFind(instance->getGraph()->getMaxNode());

    // Find all edges and sort them
    auto edgeIt = instance->getGraph()->findEdges();
    while(edgeIt.hasElement()) {
        edges.push_back(*edgeIt);
        ++edgeIt;
    }
    sort(edges.begin(), edges.end());

    // Iterate over edges in increasing cost
    for(auto i=0; i < edges.size(); i++) {
        // Find end index where costs change
        size_t j = i+1;
        for(; j < edges.size() && edges[i].cost == edges[j].cost; j++);
        // check all edges
        for(size_t k=i; k < j; k++) {
            auto& e = edges[k];
            if (e.u < instance->getNumTerminals() && e.v < instance->getNumTerminals()) {
                if (components.find(e.u) != components.find(e.v)) {
                    preselect(e.u, e.v, e.cost);
                    instance->contractEdge(e.u, e.v, &contracted);

                    // This messes up the structures so much, quit here
                    instance->setApproximationState(SteinerInstance::higher);
                    instance->setDistanceState(SteinerInstance::higher);
                    instance->setSteinerDistanceState(SteinerInstance::higher);
                    return 1;
                }
            }
        }

        // Union in separate loop to avoid false positives
        for(; i < j; i++) {
            auto& e = edges[i];
            components.unionize(e.u, e.v);
        }
    }
    enabled = false;
    return 0;
}
