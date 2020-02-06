//
// Created by aschidler on 1/28/20.
//

#include "SdcReduction.h"

node_id steiner::SdcReduction::reduce(node_id currCount, node_id prevCount) {
    auto it = instance->getGraph()->findEdges();
    node_id track = 0;

    while(it.hasElement()) {
        auto e = *it;
        auto sl = SteinerLength::calculateSteinerLength(e.u, e.v, instance->getGraph(),
            e.cost + 1, depthLimit_, true, instance->getNumTerminals(), instance->getGraph()->getMaxNode());

        if (e.cost >= sl) {
            it = instance->removeEdge(it);
            track++;
        }
        else
            ++it;
    }

    if (track > 0)
        instance->setDistanceState(SteinerInstance::higher);
    enabled = track > 0;
    return track;
}
