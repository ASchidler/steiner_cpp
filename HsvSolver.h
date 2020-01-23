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

using namespace std;

// TODO: Since all bitsets have the same size, can't we do this more statically?
namespace steiner {
    class HsvSolver {
    public:
        HsvSolver(SteinerInstance *instance);

        ~HsvSolver() {
            delete[] costs_;
        }

        Graph* solver();

    private:
        SteinerInstance *instance_;
        HashSetLabelStore store_;
        unsigned int root_;
        unordered_map<unsigned int, unsigned int> tmap_;
        unordered_set<unsigned int> terminals_;
        unsigned int nTerminals_;

        struct QueueEntry {
            QueueEntry(unsigned int cost, unsigned int node, dynamic_bitset<> label) : cost(cost), node(node),
                                                                                       label(std::move(label)) {

            }

            unsigned int cost;
            unsigned int node;
            dynamic_bitset<> label;
            bool operator<(const QueueEntry& p2) const
            {
                return cost > p2.cost || (cost == p2.cost &&
                                             ((node > p2.node) ||
                                              (node == p2.node && label < p2.label))
                );
            }
        };

        priority_queue<QueueEntry> queue_;

        union Predecessor {
            unsigned int node;
            const dynamic_bitset<> *label;
        };

        struct CostInfo {
            CostInfo(unsigned int cost, Predecessor prev, bool merge) : cost(cost), prev(prev), merge(merge) {
            }

            unsigned int cost;
            Predecessor prev;
            bool merge;
        };

        unordered_map<dynamic_bitset<>, CostInfo> *costs_;

        void process_neighbors(unsigned int n, dynamic_bitset<> *label, unsigned int cost);

        void process_labels(unsigned int n, dynamic_bitset<> *label, unsigned int cost);
    };
}

#endif //STEINER_HSVSOLVER_H
