//
// Created by aschidler on 1/29/20.
//

#include "TerminalDistanceReduction.h"

node_id steiner::TerminalDistanceReduction::reduce(node_id currCount, node_id prevCount) {
    instance->requestDistanceState(SteinerInstance::higher);
    cost_id cMax = 0;
    for(node_id t=0; t < instance->getNumTerminals(); t++) {
        auto cCost = (instance->getClosestTerminals(t) + 1)->cost;
        if (cCost > cMax)
            cMax = cCost;
    }

    vector<Edge> del;
    node_id track = 0;
    auto it = instance->getGraph()->findEdges();
    while(it.hasNext()) {
        auto e = *it;
        if (e.cost > cMax)
            del.push_back(e);

        ++it;
    }

    for(auto& e: del) {
        instance->removeEdge(e.u, e.v);
        track++;
    }

    return track;
}
