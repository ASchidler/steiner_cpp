//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_STEINERHEURISTIC_H
#define STEINER_STEINERHEURISTIC_H
#include "boost/dynamic_bitset.hpp"
using namespace boost;

namespace steiner {
    class SteinerHeuristic {
    public:
        virtual unsigned int calculate(unsigned int n, dynamic_bitset<> *label) = 0;
    };
}
#endif //STEINER_STEINERHEURISTIC_H
