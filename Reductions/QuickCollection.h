//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_QUICKCOLLECTION_H
#define STEINER_QUICKCOLLECTION_H

#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "DegreeReduction.h"
#include "ShortLinksPreselection.h"
#include "NearestVertexPreselection.h"

namespace  steiner {
    class QuickCollection : public Reduction {
    public:
        explicit QuickCollection(SteinerInstance *s) : Reduction(s), deg_(DegreeReduction(s, false)), nv_(NearestVertexPreselection(s)), sl_(ShortLinksPreselection(s)) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override {
            node_id track = 0;
            node_id thisRun = 1;
            while (thisRun > 0) {
                thisRun = 0;
                thisRun += nv_.reduce(currCount, prevCount);
                thisRun += sl_.reduce(currCount, prevCount);

                if (thisRun > 0 || track == 0)
                    thisRun += deg_.reduce(currCount, prevCount);
                track += thisRun;
            }

            return track;
        }

        bool postProcess(SteinerResult *solution) override {
            bool result1 = nv_.postProcess(solution);
            bool result2 = sl_.postProcess(solution);
            bool result3 = deg_.postProcess(solution);

            return result1 || result2 || result3;
        }

        string getName() override {
            return "Quick Reduction";
        }
    private:
        DegreeReduction deg_;
        ShortLinksPreselection sl_;
        NearestVertexPreselection nv_;
    }
    ;
}

#endif //STEINER_QUICKCOLLECTION_H
