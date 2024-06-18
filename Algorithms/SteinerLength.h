//
// Created on 1/28/20.
//

#ifndef STEINER_STEINERLENGTH_H
#define STEINER_STEINERLENGTH_H
#include "../SteinerInstance.h"
#include "../Graph.h"

namespace steiner {
    class SteinerLength {
    public:
        static cost_id calculateSteinerLength(node_id u, node_id v, Graph *g, cost_id cut_off, node_id depth_limit,
                                              bool restrict, node_id nTerminals, node_id nNodes);

    private:
        inline static void
        calculateSteinerLengthSub(node_id u, node_id v, Graph *g, cost_id cut_off, node_id depth_limit,
                                  node_id nTerminals, node_id nNodes, cost_id *scanned);
    };
}


#endif //STEINER_STEINERLENGTH_H
