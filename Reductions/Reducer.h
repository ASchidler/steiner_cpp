//
// Created on 1/27/20.
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
        static Reducer getMinimalReducer(SteinerInstance* s, bool useSl) {
            auto reductions = vector<Reduction*>();
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new LongEdgeReduction(s, true, 100));
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new SdcReduction(s, 200));
            reductions.push_back(new DegreeReduction(s, false));
            // This causes problems in rare cases
            // reductions.push_back(new Degree3Reduction(s));
            reductions.push_back(new DegreeReduction(s, false));
            reductions.push_back(new NtdkReduction(s, 200, false, 4));
            reductions.push_back(new QuickCollection(s, useSl));

            auto r = Reducer(reductions, s);
            r.setLimit(3);
            r.setSilent();
            return r;
        }
        void unreduce(SteinerResult* solution);

        void setLimit(unsigned int limit) {
            limit_ = limit;
        }

        void reset() {
            for(const auto& r : reductions_) {
                r->reset();
            }
        }

        void setSilent() {
            silent_ = true;
        }

    private:
        vector<Reduction*> reductions_;
        SteinerInstance* instance_;
        unsigned int limit_ = UINT_MAX;
        bool silent_ = false;
    };
}


#endif //STEINER_REDUCER_H
