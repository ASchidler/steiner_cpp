//
// Created by aschidler on 1/27/20.
//

#ifndef STEINER_REDUCER_H
#define STEINER_REDUCER_H
#include <bits/stdc++.h>

#include <utility>
#include "Reduction.h"
#include "../Steiner.h"
#include "../SteinerInstance.h"
#include "../SteinerTree.h"

using namespace std;

namespace steiner {
/**
 * Reduces an instance. Wraps the functionality of many Reductions
 */
    class Reducer {
    public:
        Reducer(vector<Reduction*> reductions, SteinerInstance* instance) : reductions_(std::move(reductions)), instance_(instance) {

        }
        void reduce();
        void unreduce(SteinerTree* solution);
    private:
        vector<Reduction*> reductions_;
        SteinerInstance* instance_;
    };
}


#endif //STEINER_REDUCER_H
