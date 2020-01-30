//
// Created by aschidler on 1/30/20.
//

#include "NearestVertexPreselection.h"

node_id steiner::NearestVertexPreselection::reduce(node_id currCount, node_id prevCount) {
    if (instance->getNumTerminals() <= 2)
        return 0;

    instance->requestDistanceState(SteinerInstance::higher);

    node_id track = 0;

    for (node_id t=0; t < instance->getNumTerminals(); t++) {
        // Get three smallest incident edges
        DoubleCostEntry e1(0, MAXCOST, MAXCOST);
        DoubleCostEntry e2(0, MAXCOST, MAXCOST);
        DoubleCostEntry e3(0, MAXCOST, MAXCOST);
        for(auto& n: instance->getGraph()->nb[t]) { // find incident
            auto closest = instance->getClosestTerminals(n.first);

            cost_id cmpVal = closest[0].cost;
            if (closest[0].node == t)
                cmpVal = closest[1].cost;
            cmpVal += n.second;

            if (n.second < e1.edgeCost || (n.second == e1.edgeCost && cmpVal < e1.totalCost)) {
                swap(e2, e3);
                swap(e1, e2);
                e1.totalCost = cmpVal;
                e1.node = n.first;
                e1.edgeCost = n.second;
            } else if (n.second <= e2.edgeCost) {
                swap(e2, e3);
                e2.node = n.first;
                e2.edgeCost = n.second;
            } else if (n.second <= e3.edgeCost) {
                e3.node = n.first;
                e3.edgeCost = n.second;
            }
        } // find incident

        cost_id overall = e1.totalCost;
        bool contract = false;

        // Either the second edge is larger than the cost to reach the next terminal
        if (e2.edgeCost >= overall)
            contract = false;
        else if (e3.edgeCost >= overall && e2.node >= instance->getNumTerminals()) {
            contract = true;
            for(auto& n: instance->getGraph()->nb[e2.node]) {
                if (n.first != t && n.second < overall) {
                    contract = false;
                    break;
                }
            }
        }
        if (contract) {
            preselect(t, e1.node, e1.edgeCost);
            instance->contractEdge(t, e1.node, &contracted);
            track++;
        }
    }

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::higher);
        instance->setApproximationState(SteinerInstance::higher);
        instance->setDistanceState(SteinerInstance::higher);
    }

    return track;
}
