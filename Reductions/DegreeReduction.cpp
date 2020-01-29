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
            auto dg = instance->getGraph()->nb[*n].size();
            // Degree 1, Terminals can be contracted, steiner-vertices deleted
            if (dg == 1) {
                if (*n >= instance->getNumTerminals()) { // Non-Terminal
                    n = instance->removeNode(n);
                    track++;
                    changed = true;
                }
                else if (ran_) { // Terminal
                    auto nb = instance->getGraph()->nb[*n].begin();
                    preselect(*n, nb->first, nb->second);
                    instance->moveTerminal(*n, nb->first);
                    n = instance->removeNode(nb->first);
                    ts++;
                    track++;
                    changed = true;
                }
                else{
                    ++n;
                }
            }
            // Non-Terminals of degree 2 can be merged away
            else if (dg == 2 and *n >= instance->getNumTerminals()) {
                auto u = instance->getGraph()->nb[*n].begin();
                auto v = instance->getGraph()->nb[*n].begin();
                v++;
                if (instance->addEdge(u->first, v->first, u->second + v->second))
                    merge(*n, u->first, v->first, u->second, v->second);

                n = instance->removeNode(n);
                track++;
                changed = true;
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
                    n = instance->contractEdge(min_nb, *n, &contracted);
                    ts++;
                } else {
                    ++n;
                }
            }
            else
            {
                ++n;
            }
        }
    }
    if (ts > 0) {
        instance->setDistanceState(SteinerInstance::higher);
        // TODO: Higher should suffice...
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setApproximationState(SteinerInstance::higher);
    }
    ran_ = true;
    return track;
}
