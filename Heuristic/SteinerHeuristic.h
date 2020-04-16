//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_STEINERHEURISTIC_H
#define STEINER_STEINERHEURISTIC_H
#include "../Steiner.h"
using namespace boost;

namespace steiner {
    class SteinerHeuristic {
    public:
        virtual ~SteinerHeuristic() {}
        virtual cost_id calculate(node_id n, const dynamic_bitset<> *label, const cost_id ub) = 0;
    };
}
#endif //STEINER_STEINERHEURISTIC_H
