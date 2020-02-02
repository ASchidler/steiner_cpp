//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_HEAVYEDGEREDUCTION_H
#define STEINER_HEAVYEDGEREDUCTION_H

#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class HeavyEdgeReduction : public Reduction {
    public:
        HeavyEdgeReduction(SteinerInstance *s, node_id limit) : Reduction(s), limit_(limit) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Heavy Edges";
        }

        bool postProcess(SteinerTree *solution) override;
    private:
        node_id limit_;
        struct Adaption {
            Adaption(node_id t, node_id n, cost_id oldCost, cost_id newCost)
                    : t(t), n(n), oldCost(oldCost), newCost(newCost) {

            }

            node_id t;
            node_id n;
            cost_id oldCost;
            cost_id newCost;
        };
        vector<Adaption> adaptions_;
    };
}


#endif //STEINER_HEAVYEDGEREDUCTION_H
