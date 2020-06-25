//
// Created by asc on 25.06.20.
//

#ifndef STEINER_HYBRIDSOLVER_H
#define STEINER_HYBRIDSOLVER_H

#include "Steiner.h"
#include "Structures/Queue.h"
#include "SteinerInstance.h"
#include "HsvSolver.h"

namespace steiner{
    template <typename T>
    class HybridSolver {
        struct CostInfo {
            CostInfo(unsigned int cost, Predecessor<T> prev, bool merge) : cost(cost), prev(prev), merge(merge) {
            }
            CostInfo() = default;
            cost_id cost = 0;
            Predecessor<T> prev = Predecessor<T>();
            bool merge = false;
        };

        struct QueueEntry {
            QueueEntry(T label, cost_id estimate) : label(label), estimate(estimate) {

            }
            T label;
            cost_id estimate;

            bool operator<(const QueueEntry& p2) const {
                return estimate < p2.estimate;
            }
            bool operator>(const QueueEntry& p2) const {
                return estimate > p2.estimate;
            }
        };

    public:

        void solve(SteinerInstance& instance) {
            // TODO: If we shrink the graph at the beginning, we can just iterate over the nodes, without getnodes?
            unordered_set<T> labels;
            unordered_map<T, unordered_map<node_id, CostInfo>> costs;
            cost_id ub = instance.getUpperBound(); // Shorthand
            Queue<QueueEntry> q(ub+1);

            T targetLabel = 1;
            for(node_id t=0; t < instance.getNumTerminals(); t++) {
                // Initialize Queue
                q.emplace(0, targetLabel, 0);
                // Initialize costs
                auto ce = costs.emplace(targetLabel, unordered_map<node_id, CostInfo>());
                auto pred = steiner::Predecessor<T>();
                pred.label = 0;
                ce.first->second.emplace(t, CostInfo(0, pred, true));
                // Increment
                targetLabel <<= 1u;
            }

            targetLabel -= 1; // Target label now contains the label for all terminals

            while(! q.empty()) {
                auto qe = q.dequeue();
                auto& ecosts = costs[qe.label];

                // Propagate costs
                Queue<NodeWithCost> pq(ub + 1);
                for(auto& ce: ecosts) {
                    pq.emplace(ce.second.cost, ce.first, ce.second.cost);
                }
                while(! pq.empty()) {
                    auto pqe = pq.dequeue();

                    // This might be expensive...
                    if (ecosts[pqe.node].cost < pqe.cost)
                        continue;

                    // Expand
                    for (auto& v : instance.getGraph()->nb[pqe.node]) {
                        cost_id nc = pqe.cost + v.second;
// TODO: Other pruning goes here
// TODO: Check lower bound for bound transgression?
                        if (nc <= ub) {
                            auto it = ecosts.find(v.first);
                            if (it == ecosts.end()) {
                                auto pred = Predecessor<T>();
                                pred.node = pqe.node;
                                ecosts.emplace(std::piecewise_construct, std::forward_as_tuple(v.first), std::forward_as_tuple(nc, pred, false));
                                pq.emplace(nc, v.first, nc);
                            } else if (it->second.cost > nc) {
                                it->second.cost = nc;
                                it->second.prev.node = pqe.node;
                                it->second.merge = false;
                                pq.emplace(nc, v.first, nc);
                            }
                        }
                    }
                }

                labels.emplace(qe.label);
                // At this point label is complete

                // Found root
                if(qe.label == targetLabel) {
                    cout << ecosts.begin()->second.cost << endl;
                    break;
                }

                // Merge
                for(T otherLabel: labels) {
                    if ((qe.label & otherLabel) == 0) {
                        T newLabel = qe.label | otherLabel;
                        auto& ocost = costs[otherLabel];
                        auto& ncost = costs.emplace(newLabel, unordered_map<node_id, CostInfo>()).first->second;

                        // merge costs
                        cost_id minCost = MAXCOST;
                        bool changed = false;
                        for(auto& ce: ecosts) {
                            auto oe = ocost.find(ce.first);
                            if (oe != ocost.end()) {
                                cost_id nc = ce.second.cost + oe->second.cost;
                                if (nc <= ub) {
                                    minCost = min(minCost, nc);

                                    // Check if other
                                    auto it = ncost.find(ce.first);
                                    if (it == ecosts.end()) {
                                        auto pred = Predecessor<T>();
                                        pred.label = newLabel;
                                        ncost.emplace(std::piecewise_construct, std::forward_as_tuple(ce.first), std::forward_as_tuple(nc, pred, true));
                                        changed = true;
                                    } else if (it->second.cost > nc) {
                                        it->second.cost = nc;
                                        it->second.prev.label = newLabel;
                                        it->second.merge = true;
                                        changed = true;
                                    }
                                }
                            }
                        }
                        if (changed) {
                            q.emplace(minCost, newLabel, minCost);
                        }
                    }
                }
            }

            cout << "Done" << endl;
        }
    };
};
#endif //STEINER_HYBRIDSOLVER_H
