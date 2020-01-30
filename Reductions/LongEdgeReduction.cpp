//
// Created by aschidler on 1/28/20.
//

#include "LongEdgeReduction.h"
// TODO: Add conditions when to stop or not try?
node_id steiner::LongEdgeReduction::reduce(node_id currCount, node_id prevCount) {
    instance->requestSteinerDistanceState(SteinerInstance::higher);
    node_id track = 0;

    auto it = instance->getGraph()->findEdges();
    vector<Edge> del; // Do not delete immediately as this would change nb while iterating over it
    vector<Edge> eql;

    // Test using SL approximation
    while(it.hasNext()) {
        auto e = *it;
        auto sl = instance->getSteinerDistance(e.u, e.v);
        if (e.cost > sl) {
            it = instance->removeEdge(it);
            //del.push_back(e);
            //++it;
        }
        else {
            // SL is unrestricted, so we do not know if it holds for equality
            if (e.cost == sl)
                eql.push_back(e);
            ++it;
        }
    }

    // Look here as well. For neighbors of terminals, the steiner distance is exact and may be more accurate than above
    for(int t=0; t < instance->getNumTerminals(); t++) {
        for (auto v: instance->getGraph()->nb[t]) {
            if (v.second > instance->getSteinerDistance(t, v.first))
                del.emplace_back(t, v.first, v.second);
        }

    }
    for(auto& e: del) {
        instance->removeEdge(e.u, e.v);
        track++;
    }

    // Compute the restricted SL. As this is more expensive, do only upon "request"
    if (handleEql_) {
        for(auto& e: eql) {
            if (instance->getGraph()->nb[e.u].count(e.v) > 0) {
                auto sd = SteinerLength::calculateSteinerLength(e.u, e.v, instance->getGraph(),
                                                                e.cost + 1, depthLimit_, true, instance->getNumTerminals(), instance->getGraph()->getMaxNode());
                assert(e.cost == instance->getGraph()->nb[e.u][e.v]);
                if (e.cost >= sd) {
                    instance->removeEdge(e.u, e.v);
                    track++;
                }
            }
        }
    }

    if (track > 0)
        // TODO: How could this happen? Removing edges cannot lead to shorter distances...
        instance->setDistanceState(SteinerInstance::lower);

    return track;
}
