//
// Created by aschidler on 1/23/20.
//

#ifndef DYNAMIC_STEINER_STEINERHEURISTIC_H
#define DYNAMIC_STEINER_STEINERHEURISTIC_H
#include "../Steiner.h"
#include <boost/dynamic_bitset.hpp>
using boost::dynamic_bitset;

namespace steiner {
    class DynamicSteinerHeuristic {
    public:
        virtual ~DynamicSteinerHeuristic() = default;
        virtual cost_id calculate(node_id n, const dynamic_bitset<> *label, cost_id ub) = 0;
    };
}
#endif //DYNAMIC_STEINER_STEINERHEURISTIC_H
