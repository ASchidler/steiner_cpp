//
// Created by aschidler on 1/23/20.
//

#include "MstHeuristic.h"

using namespace steiner;

cost_id MstHeuristic::calculate(node_id n, dynamic_bitset<> *label) {
    //Special case where only on terminal left...
    if (label->count() == terminals_->size())
        return instance_->getGraph()->getDistances()[root_][n];

    auto result = cache_.find(*label);

    //TODO: Precalc closest terminals
    auto ts = std::vector<node_id>();
    ts.push_back(root_);
    for (auto t: *terminals_) {
        if(!label->test((*tmap_)[t])) {
            ts.push_back(t);
        }
    }

    cost_id cost = 0;
    if (result != cache_.end())
        cost = result->second;
    else {
        cost = calcMst(ts);
    }

    cost_id minVal[2];
    int j = 0;

    // two closest distances between n and terminals
    auto closest = (NodeWithCost*) instance_->getClosestTerminals(n);
    for(int i=0; j < 2 ;i++) {
        auto nb = closest[i];

        if(nb.node == root_ || !label->test((*tmap_)[nb.node])) {
            minVal[j] = nb.cost;
            j++;
        }
    }
    return (minVal[0] + minVal[1] + cost) / 2;
}

cost_id MstHeuristic::calcMst(vector<node_id>& ts) {
    cost_id minEdge[ts.size()];
    bool taken[ts.size()];
    cost_id val;
    int idx = -1;
    cost_id sumEdges = 0;

    for(size_t i=0; i < ts.size(); i++) {
        minEdge[i] = MAXCOST;
        taken[i] = false;
    }

    // Init
    minEdge[0] = 0;

    for(int i=0; i < ts.size(); i++) {
        val = MAXCOST;
        for(int k=0; k < ts.size(); k++) {
            if (minEdge[k] < val) {
                val = minEdge[k];
                idx = k;
            }
        }

        taken[idx] = true;
        minEdge[idx] = MAXCOST;
        sumEdges += val;

        for(int k=0; k < ts.size(); k++) {
            if (! taken[k]) {
                auto dist = instance_->getGraph()->getDistances()[ts[idx]][ts[k]];
                if (dist < minEdge[k])
                    minEdge[k] = dist;
            }
        }
    }

    return sumEdges;
}