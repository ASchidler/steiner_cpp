//
// Created by aschidler on 1/28/20.
//

#ifndef STEINER_LONGEDGEREDUCTION_H
#define STEINER_LONGEDGEREDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class LongEdgeReduction : public Reduction {
    public:
        LongEdgeReduction(SteinerInstance* s, bool handleEql, node_id depthLimit) : Reduction(s), handleEql_(handleEql),
        depthLimit_(depthLimit){

        }
        node_id reduce(node_id currCount, node_id prevCount) override;
        string getName() override {
            return "Long Edges";
        }
    private:
        bool handleEql_;
        node_id depthLimit_;
    };
}


#endif //STEINER_LONGEDGEREDUCTION_H
