//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_ZEROEDGEPRESELECTION_H
#define STEINER_ZEROEDGEPRESELECTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class ZeroEdgePreselection : public Reduction {
    public:
        explicit ZeroEdgePreselection(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Zero Edge";
        }
    private:
        bool ran_ = false;
    };
}

#endif //STEINER_ZEROEDGEPRESELECTION_H
