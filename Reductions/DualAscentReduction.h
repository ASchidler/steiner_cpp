//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_DUALASCENTREDUCTION_H
#define STEINER_DUALASCENTREDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "../Algorithms/DualAscent.h"

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
        void reduceGraph(DualAscentResult& r);

    };
}


#endif //STEINER_DUALASCENTREDUCTION_H
