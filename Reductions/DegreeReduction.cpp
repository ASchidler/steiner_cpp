//
// Created by andre on 25.01.20.
//

#include "DegreeReduction.h"

using namespace steiner;

node_id DegreeReduction::reduce(node_id currCount, node_id prevCount) {
    node_id track = 0;
    node_id ts = 0;
    bool changed = true;

    while (changed) {
        changed = false;

        auto n = instance->getGraph()->getNodes()->begin();
        while(n != instance->getGraph()->getNodes()->end()) {
            bool skip = false;
            auto dg = instance->getGraph()->nb[*n].size();
            // Degree 1, Terminals can be contracted, steiner-vertices deleted
            if (dg == 1) {
                if (*n >= instance->getNumTerminals()) { // Non-Terminal
                    n = instance->removeNode(n);
                    track++;
                    changed = true;
                    skip = true;
                }
                else if (ran_) { // Terminal
                    auto nb = instance->getGraph()->nb[*n].begin();
                    auto v = nb->first; // We change nb afterwards, store v as the iterator becomes invalid
                    preselect(*n, nb->first, nb->second);
                    n = instance->contractEdge(n, v, &contracted);
                    skip = true;
                    ts++;
                    track++;
                    changed = true;
                }
            }
            // Non-Terminals of degree 2 can be merged away
            else if (dg == 2 && *n >= instance->getNumTerminals()) {
                auto u = instance->getGraph()->nb[*n].begin();
                auto v = instance->getGraph()->nb[*n].begin();
                v++;

                if (instance->addEdge(u->first, v->first, u->second + v->second))
                    merge(*n, u->first, v->first, u->second, v->second);

                n = instance->removeNode(n);
                track++;
                changed = true;
                skip = true;
            }
            // If the closest neighbor of a terminal is a terminal, can be contracted
            else if (ran_ && dg >= 2 and *n < instance->getNumTerminals()) {
                cost_id min_cost = MAXCOST;
                node_id min_nb = 0;
                for (auto& n2 : instance->getGraph()->nb[*n]) {
                    if (n2.second < min_cost || (n2.second == min_cost && n2.first < instance->getNumTerminals())) {
                        min_cost = n2.second;
                        min_nb = n2.first;
                    }
                }

                if (min_nb < instance->getNumTerminals()) {
                    preselect(*n, min_nb, min_cost);
                    n = instance->contractEdge(n, min_nb, &contracted);
                    ts++;
                    skip = true;
                    track++;
                    changed=true;
                }
            }

            if (! skip)
                ++n;
        }
    }
    if (ts > 0) {
        instance->setDistanceState(SteinerInstance::higher);
        instance->setSteinerDistanceState(SteinerInstance::higher);
        instance->setApproximationState(SteinerInstance::higher);
    }
    ran_ = true;
    return track;
}
