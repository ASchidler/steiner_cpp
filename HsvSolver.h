//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_HSVSOLVER_H
#define STEINER_HSVSOLVER_H

#include <utility>

#include "SteinerInstance.h"
#include "LabelStore.h"
#include "HashSetLabelStore.h"
#include "LabelIterator.h"
#include "Heuristic/MstHeuristic.h"
#include "Algorithms/DualAscent.h"
#include "Algorithms/ShortestPath.h"
#include "Heuristic/DualAscentHeuristic.h"
#include "Steiner.h"
#include "SteinerTree.h"
#include "Structures/Queue.h"

using namespace std;

// TODO: Since all bitsets have the same size, can't we do this more statically?
namespace steiner {
    class HsvSolver {
    public:
        explicit HsvSolver(SteinerInstance *instance);

        ~HsvSolver() {
            delete[] costs_;
            delete heuristic_;
            delete store_;
        }

        SteinerResult* solve();

    private:
        SteinerInstance *instance_;
        LabelStore* store_;
        node_id root_;
        node_id nTerminals_;

        struct QueueEntry {
            QueueEntry(cost_id cost, cost_id originalCost, node_id node, dynamic_bitset<> label) : cost(cost), node(node),
                                                                                       label(std::move(label)), originalCost(originalCost) {

            }

            cost_id cost;
            cost_id originalCost;
            node_id node;
            dynamic_bitset<> label;
            bool operator<(const QueueEntry& p2) const
            {
                return cost > p2.cost || (cost == p2.cost &&
                                             ((originalCost > p2.originalCost) ||
                                              (originalCost == p2.originalCost && node > p2.node))
                );
            }
            bool operator>(const QueueEntry& p2) const
            {
                return cost > p2.cost || (cost == p2.cost &&
                                          ((originalCost > p2.originalCost) ||
                                           (originalCost == p2.originalCost && node > p2.node))
                );
            }
        };

        Queue<QueueEntry> queue_;

        union Predecessor {
            node_id node;
            const dynamic_bitset<> *label;
        };

        struct CostInfo {
            CostInfo(unsigned int cost, Predecessor prev, bool merge) : cost(cost), prev(prev), merge(merge) {
            }
            CostInfo() = default;
            cost_id cost = 0;
            Predecessor prev = Predecessor();
            bool merge = false;
        };

        struct PruneBoundEntry {
            PruneBoundEntry(cost_id pcost, dynamic_bitset<> plabel) : cost(pcost), label(std::move(plabel)) {
            }
            cost_id cost;
            dynamic_bitset<> label;
        };

        struct PruneDistEntry {
            PruneDistEntry(cost_id cost, node_id terminal) : cost(cost), terminal(terminal) {
            }

            cost_id cost;
            node_id terminal;
        };

        unordered_map<dynamic_bitset<>, CostInfo>* costs_;
        unordered_map<dynamic_bitset<>, PruneBoundEntry> pruneBoundCache;
        unordered_map<dynamic_bitset<>, PruneDistEntry> pruneDistCache;
        SteinerHeuristic* heuristic_;

        inline void process_neighbors(node_id n, const dynamic_bitset<> *label, cost_id cost);
        inline void process_labels(node_id n, const dynamic_bitset<> *label, cost_id cost);

        bool prune(node_id n, cost_id cost, const dynamic_bitset<>* label);
        bool prune(node_id n, cost_id cost, const dynamic_bitset<>* label1, const dynamic_bitset<>* label2, dynamic_bitset<>* combined);
        inline void prune_check_bound(node_id n, cost_id cost, const dynamic_bitset<>* label);
        inline unordered_map<dynamic_bitset<>, PruneDistEntry>::iterator prune_compute_dist(const dynamic_bitset<>* label);
        inline cost_id prune_combine(const dynamic_bitset<>* label1, const dynamic_bitset<>* label2, dynamic_bitset<> *combined);

        SteinerResult* backTrack();
        void backTrackSub(node_id n, const dynamic_bitset<>* label, SteinerResult* result);
    };
}

#endif //STEINER_HSVSOLVER_H
