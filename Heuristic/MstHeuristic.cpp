//
// Created by aschidler on 1/23/20.
//

#include "MstHeuristic.h"

using namespace steiner;

unsigned int MstHeuristic::calculate(unsigned int n, dynamic_bitset<> *label) {
    auto opposite = (~*label);
    //Special case where only on terminal left...
    if (opposite.empty())
        return g_->getDistances()[root_][n];

    auto result = cache_.find(*label);

    //TODO: Precalc closest terminals
    auto ts = std::vector<unsigned int>();
    ts.push_back(root_);
    for (auto t: *terminals_) {
        if(!(opposite.test((*tmap_)[t]))) {
            ts.push_back(t);
        }
    }

    unsigned int cost = 0;
    if (result != cache_.end())
        cost = result->second;
    else {
        cost = calcMst(ts);
    }

    unsigned int minVal[] = {UINT_MAX, UINT_MAX};
    for(auto t: ts) {
        auto dist = g_->getDistances()[t][n];
        if (dist < minVal[0]) {
            minVal[1] = minVal[0];
            minVal[0] = dist;
        } else if (dist == minVal[0] || dist < minVal[1]) {
            minVal[1] = dist;
        }

        return (minVal[0] + minVal[1] + cost) / 2;
    }

}

unsigned int MstHeuristic::calcMst(vector<unsigned int>& ts) {
    unsigned int minEdge[ts.size()];
    bool taken[ts.size()];
    unsigned int val;
    int idx = -1;
    unsigned int sumEdges = 0;

    for(size_t i=0; i < ts.size(); i++) {
        minEdge[i] = UINT_MAX;
        taken[i] = false;
    }

    // Init
    minEdge[0] = 0;

    for(int i=0; i < ts.size(); i++) {
        val = UINT_MAX;
        for(int k=0; k < ts.size(); k++) {
            if (minEdge[k] < val) {
                val = minEdge[k];
                idx = k;
            }
        }

        taken[idx] = true;
        minEdge[idx] = UINT_MAX;
        sumEdges += val;

        for(int k=0; k < ts.size(); k++) {
            if (! taken[k]) {
                auto dist = g_->getDistances()[ts[idx]][ts[k]];
                if (dist < minEdge[k])
                    minEdge[k] = dist;
            }
        }
    }

    return sumEdges;
}