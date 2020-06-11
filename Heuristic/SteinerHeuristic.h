//
// Created on 1/23/20.
//

#ifndef STEINER_STEINERHEURISTIC_H
#define STEINER_STEINERHEURISTIC_H
#include "../Steiner.h"
using namespace boost;

namespace steiner {
    template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2 = nullptr>
    class SteinerHeuristic {
    public:
        virtual ~SteinerHeuristic() {}
        virtual cost_id calculate(node_id n, const T label, const cost_id ub) = 0;
    };
}
#endif //STEINER_STEINERHEURISTIC_H
