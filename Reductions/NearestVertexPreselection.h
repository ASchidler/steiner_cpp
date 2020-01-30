//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_NEARESTVERTEXPRESELECTION_H
#define STEINER_NEARESTVERTEXPRESELECTION_H

#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class NearestVertexPreselection : public Reduction {
    public:
        explicit NearestVertexPreselection(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Nearest Vertex";
        }
    };
}


#endif //STEINER_NEARESTVERTEXPRESELECTION_H
