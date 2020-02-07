//
// Created by aschidler on 1/27/20.
//

#include "Reducer.h"

void steiner::Reducer::reduce() {
    bool changed = true;
    node_id cnt = 0;
    node_id prevCnt = 0;
    unsigned int runs = 0;
    while(changed && runs < limit_) {
        for(auto reduction: reductions_) {
            if (reduction->enabled) {
                auto result = reduction->reduce(cnt, prevCnt);
                cnt += result;
                if (! silent_)
                    cout << reduction->getName() << " " << result << endl;
                for (node_id t = 0; t < instance_->getNumTerminals(); t++) {
                    assert(instance_->getGraph()->getNodes().count(t) > 0);
                }
                assert(instance_->getGraph()->checkConnectedness(instance_->getNumTerminals(), false));
            }
        }
        runs++;
        instance_->shrink();

        changed = cnt > 0;
        prevCnt = cnt;
        cnt = 0;
    }
}

void steiner::Reducer::unreduce(SteinerResult* solution) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (auto reduction : reductions_) {
            if (reduction->postProcess(solution))
                changed = true;
        }
    }
    solution->cost = solution->g->getCost();
}
