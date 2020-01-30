//
// Created by aschidler on 1/30/20.
//

#include "HeavyEdgeReduction.h"

node_id steiner::HeavyEdgeReduction::reduce(node_id currCount, node_id prevCount) {
    // Run only once!
    if (! ran_)
        return 0;
    ran_ = true;

    node_id track = 0;
    for(node_id t=0; t < instance->getNumTerminals(); t++) {
        auto nb = instance->getGraph()->nb[t];
        if (nb.size() < 2)
            continue;

        // Check if all incident edges have the same weight
        auto it = nb.begin();
        cost_id c = it->second;
        bool same = true;
        ++it;
        while(it != nb.end()) {
            if (it->second != c)
                same = false;
            ++it;
        }

        if (! same)
            continue;

        cost_id maxDist = 0;
        for (auto& n1: nb) {
            for(auto& n2: nb) {
                if (n1.first < n2.first) {
                    auto sl =SteinerLength::calculateSteinerLength(n1.first, n2.first, instance->getGraph(), c, limit_, false, instance->getNumTerminals(), instance->getGraph()->getMaxNode());
                    maxDist = max(maxDist, sl);
                }
            }
        }

        if (maxDist >= c - 1)
            continue;

        auto rt = instance->getGraph()->getReverseMapping(t);
        for(auto& n : nb) {
            adaptions_.emplace_back(rt, instance->getGraph()->getReverseMapping(n.first), n.second, maxDist + 1);
            instance->getGraph()->nb[t][n.first] = maxDist + 1;
            instance->getGraph()->nb[n.first][t] = maxDist + 1;
        }
        track++;
    }

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setDistanceState(SteinerInstance::invalid);
        instance->setApproximationState(SteinerInstance::invalid);
    }

    return track;
}

bool steiner::HeavyEdgeReduction::postProcess(steiner::SteinerTree *solution) {
    bool changed = false;
    for(auto& a: adaptions_) {
        if (solution->adaptWeight(a.t, a.n, a.oldCost, a.newCost))
            changed = true;
    }
    bool changed2 = Reduction::postProcess(solution);
    return  changed2 || changed;
}
