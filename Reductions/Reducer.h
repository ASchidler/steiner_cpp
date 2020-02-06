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
#include "Reducer.h"
#include "DegreeReduction.h"
#include "LongEdgeReduction.h"
#include "SdcReduction.h"
#include "NtdkReduction.h"
#include "TerminalDistanceReduction.h"
#include "Degree3Reduction.h"
#include "DualAscentReduction.h"
#include "HeavyEdgeReduction.h"
#include "ZeroEdgePreselection.h"
#include "MstPreselection.h"
#include "ShortLinksPreselection.h"
#include "NearestVertexPreselection.h"
#include "QuickCollection.h"

using namespace std;

namespace steiner {
/**
 * Reduces an instance. Wraps the functionality of many Reductions
 */
    class Reducer {
    public:
        Reducer(vector<Reduction*> reductions, SteinerInstance* instance) : reductions_(std::move(reductions)), instance_(instance) {

        }
        ~Reducer() {
            for(auto r: reductions_)
                delete r;
        }
        void reduce();
        static Reducer getMinimalReducer(SteinerInstance* s) {
            auto reductions = vector<Reduction*>();
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new LongEdgeReduction(s, true, 100));
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new NtdkReduction(s, 100, true, 4));
            reductions.push_back(new SdcReduction(s, 100));
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new Degree3Reduction(s));
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new NtdkReduction(s, 2000, false, 4));
            reductions.push_back(new QuickCollection(s));

            auto r = Reducer(reductions, s);
            r.setLimit(3);
            return r;
        }
        void unreduce(SteinerResult* solution);

        void setLimit(unsigned int limit) {
            limit_ = limit;
        }

    private:
        vector<Reduction*> reductions_;
        SteinerInstance* instance_;
        unsigned int limit_ = UINT_MAX;
    };
}


#endif //STEINER_REDUCER_H
