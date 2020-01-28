//
// Created by aschidler on 1/24/20.
//

#ifndef STEINER_SHORTESTPATH_H
#define STEINER_SHORTESTPATH_H

#include "../SteinerInstance.h"
#include "../SteinerTree.h"

namespace steiner {
    // TODO: Add local improvement
    struct SPHEntry {
        SPHEntry(node_id node, cost_id totalCost, cost_id edgeCost) : node(node), totalCost(totalCost), edgeCost(edgeCost) {
        }

        node_id node;
        cost_id totalCost;
        cost_id edgeCost;

        // TODO: These are actually the wrong way so that priority queues are min queues...
        bool operator<(const SPHEntry& p2) const {
            return totalCost > p2.totalCost ||
            // Choosing shorter edges over longer ones often provides better results
            (totalCost == p2.totalCost && edgeCost < p2.edgeCost) ||
            // This is just a tie breaker
            (totalCost == p2.totalCost && edgeCost == p2.edgeCost && node < p2.node);
        }
    };

    class ShortestPath {
    public:
        static SteinerTree *calculate(node_id root, Graph* g, node_id nTerminals, node_id nNodes);
        static bool hasRun;
        static node_id bestRoot;
        static cost_id bestResult;
    };
}


#endif //STEINER_SHORTESTPATH_H
