//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_STEINERHEURISTIC_H
#define STEINER_STEINERHEURISTIC_H
#include "boost/dynamic_bitset.hpp"
#include "../steiner.h"
using namespace boost;

namespace steiner {
    class SteinerHeuristic {
    public:
        virtual ~SteinerHeuristic() {}
        virtual cost_id calculate(node_id n, dynamic_bitset<> *label) = 0;
    };
}
#endif //STEINER_STEINERHEURISTIC_H
