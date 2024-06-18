//
// Created on 1/28/20.
//

#ifndef STEINER_SDCREDUCTION_H
#define STEINER_SDCREDUCTION_H

#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class SdcReduction : public Reduction {
    public:
        SdcReduction(SteinerInstance *s, node_id depthLimit) : Reduction(s),
                                                                               depthLimit_(depthLimit) {

        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "SDC";
        }

    private:
        node_id depthLimit_;
    };
}

#endif //STEINER_SDCREDUCTION_H
