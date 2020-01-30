//
// Created by aschidler on 1/30/20.
//

#include "DualAscentReduction.h"

node_id steiner::DualAscentReduction::reduce(node_id currCount, node_id prevCount) {

    if (instance->getNumTerminals() < 3)
        return 0;

    // Always request a new approximation for this reduction
    instance->requestApproximationState(SteinerInstance::invalid);

    // TODO: Define control parameters according to instance size

    // TODO: Root selection
    // TODO: Allow for non-terminal roots?
    node_id numRoots = min((node_id)10, instance->getNumTerminals());
    //node_id roots[numRoots];
//    DualAscent* results[numRoots];
//    for(node_id t=0; t < numRoots; t++)
//        results[t] = DualAscent::




}

void steiner::DualAscentReduction::reduceGraph(steiner::DualAscentResult &r) {

}
