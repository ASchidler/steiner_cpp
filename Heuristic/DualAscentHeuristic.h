//
// Created on 1/23/20.
//

#ifndef STEINER_DUALASCENTHEURISTIC_H
#define STEINER_DUALASCENTHEURISTIC_H
#include "../SteinerInstance.h"
#include "SteinerHeuristic.h"
#include "../SteinerInstance.h"
#include "../HashSetLabelStore.h"

using namespace steiner;
using namespace boost;

namespace steiner {
    template <typename T>
    class DualAscentHeuristic : public SteinerHeuristic<T> {
    public:
        DualAscentHeuristic(SteinerInstance* instance, node_id root, node_id nTerminals, node_id numNodes, T maxTerminal)
        : instance_(instance), root_(root), nTerminals_(nTerminals), nNodes_(numNodes), maxTerminal_(maxTerminal) {

        }

        ~DualAscentHeuristic() {
            for (const auto &elem: cache_) {
                delete[] elem.second;
            }
        }

        cost_id calculate(node_id n, const T label, const cost_id ub);

    private:
        SteinerInstance* instance_;
        node_id root_;
        unordered_map<T, cost_id*> cache_;
        node_id nTerminals_;
        node_id nNodes_;
        T maxTerminal_;

        cost_id *precalculate(const T label, const cost_id ub);
    };
}

#endif //STEINER_DUALASCENTHEURISTIC_H
