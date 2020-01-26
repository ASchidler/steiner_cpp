//
// Created by andre on 25.01.20.
//

#include "DegreeReduction.h"

using namespace steiner;

node_id DegreeReduction::reduce(steiner::SteinerInstance *instance, node_id currCount, node_id prevCount) {
    node_id track = 0;
    node_id ts = 0;

    node_id old = instance->getGraph()->getNumNodes() + 1;

    while (old > instance->getGraph()->getNumNodes()) {
        old = instance->getGraph()->getNumNodes();

        for(auto n: *instance->getGraph()->getNodes()) {
            auto dg = instance->getGraph()->nb[n].size();
            // Degree 1, Terminals can be contracted, steiner-vertices deleted
            if (dg == 1) {
                if (instance->getTerminals()->count(n) == 0) {
                    instance->getGraph()->removeNode(n);
                } else if (ran_) {
                    auto nb = instance->getGraph()->nb[n].begin();
                    contracted.emplace_back(n, nb->first, nb->second);
                    instance->getGraph()->removeNode(n);
                    ts++;
                }
            }
        }
    }
}

void DegreeReduction::postProcess(steiner::Graph *solution) {

}