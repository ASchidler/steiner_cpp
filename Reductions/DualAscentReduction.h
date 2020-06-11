//
// Created on 1/30/20.
//

#ifndef STEINER_DUALASCENTREDUCTION_H
#define STEINER_DUALASCENTREDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "../Algorithms/DualAscent.h"
#include <random>
#include "../Algorithms/VoronoiDiagram.h"

namespace  steiner {
    class DualAscentReduction : public Reduction {
    public:
        explicit DualAscentReduction(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Dual Ascent";
        }
    private:
        node_id reduceGraph(SteinerResult* r, VoronoiDiagram& vor, SteinerInstance* inst, cost_id bound);
        void prune(steiner::SteinerResult *r);
        bool prewarning_ = false;

        node_id bestRoots[2] = {0, 1};

        void chooseRoots(node_id* roots, node_id numRoots);
        void selectRoots(SteinerResult** results, node_id numSolutions, const node_id* track);
        void pruneAscent(SteinerResult** results, node_id numSolutions, node_id numRuns);

        node_id reduceGraphNtdk(SteinerResult *r, VoronoiDiagram &vor, SteinerInstance *inst, cost_id bound, Graph& cumulative);
    };
}


#endif //STEINER_DUALASCENTREDUCTION_H
