//
// Created by aschidler on 1/23/20.
//

#include "MstHeuristic.h"

using namespace steiner;

cost_id MstHeuristic::calculate(node_id n, dynamic_bitset<> *label) {
    //Special case where only root left
    if (label->count() == terminals_->size())
        return instance_->getGraph()->getDistances()[root_][n];

    auto result = cache_.find(*label);

    cost_id cost = 0;
    if (result != cache_.end())
        cost = result->second;
    else {
        cost = calcMst(label);
    }

    // two closest distances between n and terminals outside the current sub-solution
    auto closest = (NodeWithCost*) instance_->getClosestTerminals(n);
    for(int j=0; j < 2 ;) {
        if(closest->node == root_ || !label->test(tmap_->at(closest->node))) {
            cost += closest->cost;
            j++;
        }
        closest++;
    }

    return cost / 2;
}

cost_id MstHeuristic::calcMst(dynamic_bitset<> *label) {
    // Find terminals outside sub-solution
    auto ts = std::vector<node_id>();
    ts.push_back(root_);
    for (auto t: *terminals_) {
        if(!label->test((*tmap_)[t])) {
            ts.push_back(t);
        }
    }

    // Calculate mst of distance graph
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

    cache_.insert(pair<dynamic_bitset<>, cost_id>(*label, sumEdges));

    return sumEdges;
}