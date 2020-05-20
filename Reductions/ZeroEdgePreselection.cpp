//
// Created by aschidler on 1/30/20.
//

#include "ZeroEdgePreselection.h"

node_id steiner::ZeroEdgePreselection::reduce(node_id currCount, node_id prevCount) {
    enabled = false;

    ran_ = true;
    node_id track = 0;
    auto it = instance->getGraph()->getNodes().begin();
    while(it != instance->getGraph()->getNodes().end()) {
        bool changed = false;
        for(auto& n: instance->getGraph()->nb[*it]) {
            if (n.second == 0) {
                it = instance->contractEdge(*it, n.first, &contracted);
                track++;
                changed = true;
                break;
            }
        }
        if (! changed)
            ++it;
    }

    return track;
}
