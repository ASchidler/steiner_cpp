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
                    instance->getGraph()->switchVertices(*n, nb->first);
                    n = instance->removeNode(nb->first);
                    ts++;
                    track++;
                    changed = true;
                }
                else{
                    ++n;
                }
            }
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
            else
            {
                ++n;
            }
        }
    }
    ran_ = true;
    return track;
}
