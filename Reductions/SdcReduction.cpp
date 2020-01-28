//
// Created by aschidler on 1/28/20.
//

#include "SdcReduction.h"

node_id steiner::SdcReduction::reduce(node_id currCount, node_id prevCount) {
    auto it = instance->getGraph()->findEdges();
    vector<Edge> del;
    node_id track = 0;

    while(it.hasNext()) {
        auto e = *it;
        auto sl = SteinerLength::calculateSteinerLength(e.u, e.v, instance->getGraph(),
                                                        e.cost + 1, depthLimit_, true, instance->getNumTerminals(), instance->getGraph()->getMaxNode());
        if (e.cost >= sl)
            del.push_back(e);
        ++it;
    }

    for(auto& e: del) {
        instance->removeEdge(e.u, e.v);
        track++;
    }

    return track;
}
