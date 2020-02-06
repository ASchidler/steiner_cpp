//
// Created by aschidler on 1/29/20.
//

#include "TerminalDistanceReduction.h"

node_id steiner::TerminalDistanceReduction::reduce(node_id currCount, node_id prevCount) {
    instance->requestDistanceState(SteinerInstance::higher);
    if (instance->getNumTerminals() < 3)
        return 0;

    // TODO: Inefficient...
    Graph g;
    for(node_id t=0; t < instance->getNumTerminals(); t++) {
        for(node_id t2 = 1; t2 < instance->getNumTerminals(); t2++) {
            if (t != t2) {
                auto& cl = instance->getClosestTerminals(t)[t2];
                g.addEdge(t, cl.node, cl.cost);
            }
        }
    }

    auto mst = g.mst();
    auto ei = mst->findEdges();
    cost_id newMax = 0;
    while(ei.hasElement()) {
        auto ce = *ei;
        newMax = max(newMax, ce.cost);
        ++ei;
    }

    node_id track = 0;
    auto it = instance->getGraph()->findEdges();
    while(it.hasElement()) {
        auto e = *it;
        if (e.cost > newMax) {
            it = instance->removeEdge(it);
            track++;
        }
        else
            ++it;
    }

    return track;
}
