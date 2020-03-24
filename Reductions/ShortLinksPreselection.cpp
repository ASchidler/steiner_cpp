//
// Created by aschidler on 1/30/20.
//

#include "ShortLinksPreselection.h"

node_id steiner::ShortLinksPreselection::reduce(node_id currCount, node_id prevCount) {
    if (instance->getNumTerminals() <= 2)
        return 0;

    instance->requestDistanceState(SteinerInstance::higher);

    node_id track = 0;
    BridgingEdge bridgingEdges[instance->getNumTerminals()];
    cost_id comparison[instance->getNumTerminals()];
    bool forbidden[instance->getNumTerminals()];
    for (node_id t=0; t < instance->getNumTerminals(); t++) {
        forbidden[t] = false;
    }

    auto it = instance->getGraph()->findEdges();

    while(it.hasElement()) {
        auto e = *it;
        auto t1 = instance->getClosestTerminals(e.u)[0].node;
        auto t2 = instance->getClosestTerminals(e.v)[0].node;

        if (t1 != t2) {
            if (e.cost < bridgingEdges[t1].c) {
                comparison[t1] = bridgingEdges[t1].c;
                bridgingEdges[t1].u = e.u;
                bridgingEdges[t1].v = e.v;
                bridgingEdges[t1].c = e.cost;
                bridgingEdges[t1].t = t2;
            } else if (e.cost < comparison[t1])
                comparison[t1] = e.cost;

            if (e.cost < bridgingEdges[t2].c) {
                comparison[t2] = bridgingEdges[t2].c;
                bridgingEdges[t2].u = e.u;
                bridgingEdges[t2].v = e.v;
                bridgingEdges[t1].t = t1;
                bridgingEdges[t2].c = e.cost;
            } else if (e.cost < comparison[t2])
                comparison[t2] = e.cost;
        }

        ++it;
    }

    for (node_id t=0; t < instance->getNumTerminals(); t++) {
        auto& e = bridgingEdges[t];

        // The graph may have changed in a previous iteration
        if (instance->getGraph()->nb[e.u].count(e.v) == 0 || e.t >= instance->getNumTerminals())
            continue;

        cost_id total = instance->getClosestTerminals(e.u)[0].cost + e.c + instance->getClosestTerminals(e.v)[0].cost;

        if (comparison[t] >= total) {
            // The idea here is as follows, whenever we contract an edge between two terminals, one gets deleted.
            // Now the cached data is stale. We therefore mark the removed terminal as forbidden, and ignore.
            // As a terminal deletion implies swapping it with the last terminal, we also forbid it.
            if (forbidden[instance->getClosestTerminals(e.u)[0].node] || forbidden[instance->getClosestTerminals(e.v)[0].node])
                continue;
            if (e.u < instance->getNumTerminals() && e.v < instance->getNumTerminals()) {
                forbidden[e.u] = true;
                forbidden[instance->getNumTerminals()-1] = true;
            }
            preselect(e.u, e.v, e.c);
            if (e.v < instance->getNumTerminals())
                instance->contractEdge(e.v, e.u, &contracted);
            else
                instance->contractEdge(e.u, e.v, &contracted);
        }
    }

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::higher);
        instance->setApproximationState(SteinerInstance::higher);
        instance->setDistanceState(SteinerInstance::higher);
    }
    return track;
}
