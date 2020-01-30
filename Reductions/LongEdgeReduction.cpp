//
// Created by aschidler on 1/28/20.
//

#include "LongEdgeReduction.h"
// TODO: Add conditions when to stop or not try?
node_id steiner::LongEdgeReduction::reduce(node_id currCount, node_id prevCount) {
    instance->requestSteinerDistanceState(SteinerInstance::higher);
    node_id track = 0;

    auto it = instance->getGraph()->findEdges();
    vector<Edge> del;
    vector<Edge> eql;

    while(it.hasNext()) {
        auto e = *it;
        auto sl = instance->getSteinerDistance(e.u, e.v);
        if (e.cost > sl)
            del.push_back(e);
        else if (e.cost == sl)
            eql.push_back(e);
        ++it;
    }

    // TODO: Can the iterator be modified to allow for deletion?
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
