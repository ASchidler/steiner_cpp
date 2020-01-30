//
// Created by aschidler on 1/29/20.
//

#ifndef STEINER_TERMINALDISTANCEREDUCTION_H
#define STEINER_TERMINALDISTANCEREDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class TerminalDistanceReduction : public Reduction {
    public:
        explicit TerminalDistanceReduction(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Terminal Distance";
        }
    };
}


#endif //STEINER_TERMINALDISTANCEREDUCTION_H
