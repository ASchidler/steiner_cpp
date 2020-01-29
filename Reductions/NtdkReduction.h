//
// Created by aschidler on 1/28/20.
//

#ifndef STEINER_NTDKREDUCTION_H
#define STEINER_NTDKREDUCTION_H

#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace steiner {
    class NtdkReduction : public Reduction {
    public:
        NtdkReduction(SteinerInstance *s, node_id depthLimit, bool restricted, node_id limit)
                : Reduction (s), restricted_(restricted), depthLimit_(depthLimit), limit_(limit) {}

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "NTDK";
        }

    private:
        node_id depthLimit_;
        bool restricted_;
        node_id limit_;
        // This is used as a lookup for which edges to take for degree 4 vertices
        // see implementation in cpp file for details
        const node_id dg4Lookup[4][3] = {
                {3, 4, 5},
                {1, 2, 5},
                {0, 2, 4},
                {0, 1, 3}
        };
    };
}


#endif //STEINER_NTDKREDUCTION_H
