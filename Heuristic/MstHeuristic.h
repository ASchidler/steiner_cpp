//
// Created on 1/23/20.
//

#ifndef STEINER_MSTHEURISTIC_H
#define STEINER_MSTHEURISTIC_H

#include "SteinerHeuristic.h"
#include "../Graph.h"
#include "../SteinerInstance.h"
// TODO: Maybe implement the STP heuristic computing STP for 2-3 terminals?
#include "../HashSetLabelStore.h"
namespace steiner {
    template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2 = nullptr>
    class MstHeuristic : public SteinerHeuristic<T> {
    public:
        MstHeuristic(SteinerInstance* instance, node_id root, node_id nTerminals, T maxTerminal)
                : instance_(instance), root_(root), nTerminals_(nTerminals), maxTerminal_(maxTerminal) {

        }
        cost_id calculate(node_id n, const T label, const cost_id ub) override;

    private:
        SteinerInstance* instance_;
        node_id root_;
        unordered_map<T, cost_id> cache_;
        node_id nTerminals_;
        T maxTerminal_;
        cost_id calcMst(const T label);
    };
}


#endif //STEINER_MSTHEURISTIC_H
