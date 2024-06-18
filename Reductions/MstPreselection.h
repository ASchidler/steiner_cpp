//
// Created on 1/30/20.
//

#ifndef STEINER_MSTPRESELECTION_H
#define STEINER_MSTPRESELECTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class MstPreselection : public Reduction {
    public:
        explicit MstPreselection(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "MST Preselection";
        }
    };
}


#endif //STEINER_MSTPRESELECTION_H
