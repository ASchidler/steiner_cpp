//
// Created on 1/22/20.
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
    template<typename T2>
    union Predecessor {
        node_id node;
        T2 label;
    };


    template <typename T>
    class HsvSolver {
    public:
        HsvSolver(SteinerInstance *instance, node_id dualAscentLimit);

        ~HsvSolver() {
            delete[] costs_;
            delete heuristic_;
            delete store_;
        }

        SteinerResult* solve();

    private:
        SteinerInstance *instance_;
        LabelStore<T>* store_;
        node_id root_;
        node_id nTerminals_;
        node_id dualAscentLimit_;

        struct QueueEntry {
            QueueEntry(cost_id cost, cost_id originalCost, node_id node, T label) : cost(cost), node(node),
                                                                                       label(label), originalCost(originalCost) {

            }

            cost_id cost;
            cost_id originalCost;
            node_id node;
            T label;
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

        struct CostInfo {
            CostInfo(unsigned int cost, Predecessor<T> prev, bool merge) : cost(cost), prev(prev), merge(merge) {
            }
            CostInfo() = default;
            cost_id cost = 0;
            Predecessor<T> prev = Predecessor<T>();
            bool merge = false;
        };

        struct PruneBoundEntry {
            PruneBoundEntry(cost_id pcost, T plabel) : cost(pcost), label(plabel) {
            }
            cost_id cost;
            T label;
        };

        struct PruneDistEntry {
            PruneDistEntry(cost_id cost, node_id terminal) : cost(cost), terminal(terminal) {
            }

            cost_id cost;
            node_id terminal;
        };

        unordered_map<T, CostInfo>* costs_;
        unordered_map<T, PruneBoundEntry> pruneBoundCache;
        unordered_map<T, PruneDistEntry> pruneDistCache;
        SteinerHeuristic<T>* heuristic_;
        T maxTerminal_;

        inline void process_neighbors(node_id n, const T label, cost_id cost);
        inline void process_labels(node_id n, const T label, cost_id cost);

        bool prune(node_id n, cost_id cost, const T label);
        bool prune(node_id n, cost_id cost, const T label1, const T label2, T combined);
        inline void prune_check_bound(node_id n, cost_id cost, const T label);
        inline typename unordered_map<T, PruneDistEntry>::iterator prune_compute_dist(const T label);
        inline cost_id prune_combine(const T label1, const T label2, T combined);

        SteinerResult* backTrack();
        void backTrackSub(node_id n, const T label, SteinerResult* result);
    };
}

#endif //STEINER_HSVSOLVER_H
