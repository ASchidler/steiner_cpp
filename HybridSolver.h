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
            CostInfo(unsigned int cost, Predecessor<T> prev, bool merge) : cost(cost), prev(prev), merge(merge), valid(true) {
            }
            CostInfo() = default;
            cost_id cost = MAXCOST;
            Predecessor<T> prev = Predecessor<T>();
            bool merge = false;
            bool valid = false;

            static bool comparePtr(CostInfo* a, CostInfo* b) { return (a->cost < b->cost); }

            // TODO: Pointer access my not be aligned
            static bool comparePairPtr(pair<CostInfo*, node_id>& a, pair<CostInfo*, node_id>& b) {
                return a.first->cost < b.first->cost;
            }

            static bool comparePairPtrT(pair<CostInfo*, node_id>& a, pair<CostInfo*, node_id>& b) {
                return a.first->cost > b.first->cost;
            }
        };

        struct QueueEntry {
            QueueEntry(T label, cost_id estimate) : label(label), estimate(estimate) {

            }
            QueueEntry(T label, cost_id estimate, uint16_t revision) : label(label), estimate(estimate), revision(revision) {

            }
            T label;
            cost_id estimate;
            uint16_t revision = 0;

            bool operator<(const QueueEntry& p2) const {
                return estimate < p2.estimate;
            }
            bool operator>(const QueueEntry& p2) const {
                return estimate > p2.estimate;
            }
        };

        struct PruneState {
            PruneState() = default;

            cost_id tp = MAXCOST;
            node_id occurance = 0;
            char sep = 0;
        };

    public:

        void propagate(SteinerInstance& instance, Queue<NodeWithCost>& pq, cost_id ub, CostInfo* costs) {
            while(! pq.empty()) {
                auto pqe = pq.dequeue();

                if (costs[pqe.node].cost < pqe.cost)
                    continue;

                // Expand
                for (auto& v : instance.getGraph()->nb[pqe.node]) {
                    cost_id nc = pqe.cost + v.second;
                    if (costs[v.first].cost > nc) {
                        costs[v.first].cost = nc;
                        costs[v.first].prev.node = pqe.node;
                        costs[v.first].merge = false;
                        // TODO: This probably causes a cache miss
                        // TODO: Other pruning goes here
                        // TODO: Check lower bound for bound transgression?
                        costs[v.first].valid = costs[pqe.node].valid;
                        if (nc > ub) {
                            costs[v.first].cost = ub + 1;
                            costs[v.first].valid = false;
                        } else {
                            pq.emplace(nc, v.first, nc);
                        }
                    }
                }
            }
        }

        void merge(unordered_map<T, uint16_t>& labels, T label, unordered_map<T, CostInfo*>& costs, Queue<QueueEntry>& q, cost_id ub, SteinerInstance& instance) {
            auto& ecosts = costs.at(label);
            for(auto& otherLabel: labels) {
                if ((label & otherLabel.first) == 0) {
                    T newLabel = label | otherLabel.first;
                    uint16_t revision = 0;
                    auto lit = labels.find(newLabel);
                    if (lit != labels.end())
                        revision = lit->second +1;

                    Queue<NodeWithCost> sq(ub + 1);
                    auto& ocost = costs.at(otherLabel.first);

                    CostInfo* ncost = nullptr;
                    auto it = costs.find(newLabel);
                    if (it != costs.end())
                        ncost = it->second;

                    // merge costs
                    cost_id minCost = MAXCOST;
                    bool change = false;
                    for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                        if (ecosts[i].valid && ocost[i].valid) {
                            cost_id nc = ecosts[i].cost + ocost[i].cost;
                            if (nc <= ub) {
                                minCost = min(minCost, nc);
                                if (ncost == nullptr) {
                                    ncost = new CostInfo[instance.getGraph()->getMaxNode()];
                                    costs.emplace(newLabel, ncost);
                                }

                                if (ncost[i].cost > nc) {
                                    change = true;
                                    ncost[i].cost = nc;
                                    ncost[i].valid = true;
                                    ncost[i].prev.label = otherLabel.first;
                                    ncost[i].merge = true;
                                }
                            }
                        }
                    }
                    if (change) {
                        // TODO: In case of consistency, re-adding is probably not necessary
                        q.emplace(minCost, newLabel, minCost, revision);
                    }
                }
            }
        }

        void solve(SteinerInstance& instance) {
            // TODO: If we shrink the graph at the beginning, we can just iterate over the nodes, without getnodes?
            unordered_map<T, uint16_t> labels;
            unordered_map<T, CostInfo*> costs;
            cost_id ub = instance.getUpperBound(); // Shorthand
            Queue<QueueEntry> q(ub+1);

            T targetLabel = 1;
            for(node_id t=0; t < instance.getNumTerminals(); t++) {
                // Initialize Queue
                q.emplace(0, targetLabel, 0);
                // Initialize costs
                auto* newCosts = new CostInfo[instance.getGraph()->getMaxNode()];
                auto ce = costs.emplace(targetLabel, newCosts);
                newCosts[t].merge = true;
                newCosts[t].valid = true;
                newCosts[t].cost = 0;
                newCosts[t].prev.label = 0;

                // Increment
                targetLabel <<= 1u;
            }
            // TODO: Cost table for labels to check if re-evaluation is necessary
            targetLabel -= 1; // Target label now contains the label for all terminals

            while(! q.empty()) {
                auto qe = q.dequeue();
                auto& ecosts = costs.at(qe.label);

                auto qit = labels.find(qe.label);
                if (qit != labels.end()) {
                    if (qit->second >= qe.revision)
                        continue;
                    else
                        qit->second = qe.revision;
                } else {
                    labels.emplace(qe.label, qe.revision);
                }

                // Found root
                if(qe.label == targetLabel) {
                    while(!ecosts->valid)
                        ecosts++;

                    cout << ecosts->cost << endl;
                    break;
                }

                Queue<NodeWithCost> sq(ub+1);
                for (node_id i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (ecosts[i].cost <= ub)
                        sq.emplace(ecosts[i].cost, i, ecosts[i].cost);
                }
                propagate(instance, sq, ub, ecosts);

                // Sort costs
                vector<pair<CostInfo*, node_id>> localCosts;
                for (node_id i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    localCosts.emplace_back(ecosts + i, i);
                }
                std::sort(localCosts.begin(), localCosts.end(), CostInfo::comparePairPtr);

                // Compare
                PruneState state[instance.getGraph()->getMaxNode()];
                cost_id maxC = 0;
                node_id cnt = 0;

                for(auto& entry: localCosts) {
                    if (entry.first->valid) {
                        cnt++;
                        maxC = max(maxC, twoPathDist(entry.second, qe.label, costs, entry.first->cost, instance, state));
                    }
                }

                Queue<NodeWithCost> qu(maxC);

                // Add all nodes present in all trees
                for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (state[i].occurance == cnt) {
                        qu.emplace(maxC - state[i].tp, i, maxC - state[i].tp);
                        state[i].sep = 1;
                    }
                }

                // Find separator candidate (N) based on two path
                while(! qu.empty()) {
                    auto nc = qu.dequeue();
                    if (state[nc.node].sep == 1)
                        continue;

                    state[nc.node].sep = 1;

                    for(const auto& v : instance.getGraph()->nb[nc.node]) {
                        if (state[v.first].sep == 0) {
                            if (v.second < nc.cost) {
                                qu.emplace(nc.cost-v.second, nc.node, nc.cost-v.second);
                            }
                        }
                    }
                }

                // Extend the candidate, until it is a separator
                std::sort(localCosts.begin(), localCosts.end(), CostInfo::comparePairPtrT);

                vector<node_id> cq;
                node_id target = instance.getNumTerminals() - std::popcount(qe.label);
                cost_id overallLimit = MAXCOST;
                bool ran = false;
                for(auto& cse: localCosts) {
                    if (target == 0)
                        break;

                    // Part of the separator
                    if (state[cse.second].sep == 1)
                        continue;

                    overallLimit = cse.first->cost;

                    state[cse.second].sep = 2;
                    if (ran) {
                        for(const auto& v : instance.getGraph()->nb[cse.second]) {
                            if (state[v.first].sep == 2)
                                cq.emplace_back(v.first);
                        }
                    } else {
                        cq.emplace_back(cse.second);
                        ran = true;
                    }

                    while (! cq.empty()) {
                        auto cqe = cq.back();
                        cq.pop_back();

                        for(const auto& v : instance.getGraph()->nb[cqe]) {
                            if (state[v.first].sep == 2) {
                                if (v.first < instance.getNumTerminals()) {
                                    T mask = 1; // This construct avoids an overflow if 128 bits are used
                                    mask <<= v.first;
                                    if((mask & qe.label) == 0)
                                        target--;
                                }

                                state[v.first].sep = 3;
                                cq.emplace_back(v.first);
                            }
                        }
                    }
                }

                for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (ecosts[i].cost > overallLimit) {
                        ecosts[i].valid = false;
                        ecosts[i].cost = overallLimit + 1;
                    }
                }

                // Merge
                merge(labels, qe.label, costs, q, ub, instance);
            }

            cout << "Done" << endl;
        }

        struct TwoPathEntry {
            TwoPathEntry(node_id r, T label, cost_id cCost, cost_id maxCost) : r(r), label(label), cCost(cCost), maxCost(maxCost) {}
            const node_id r;
            const T label;
            cost_id cCost;
            cost_id maxCost;
        };

        inline cost_id twoPathDist(node_id r, const T label, unordered_map<T,
                CostInfo*>& costs, cost_id cost, SteinerInstance& instance, PruneState* state) {
            cost_id maxCost = 0;

            state[r].occurance += 1;

            vector<TwoPathEntry> q;
            q.emplace_back(r, label, 0, 0);

            CostInfo* mainCosts = costs.at(label);
            CostInfo* cCosts = mainCosts;
            auto cLabel = label;

            bool isValid = mainCosts[r].valid;

            if (isValid)
                state[r].tp = 0;

            while(! q.empty()) {
                auto cEntry = q.back();
                q.pop_back();

                if(mainCosts[cEntry.r].cost > cost) {
                    mainCosts[cEntry.r].valid = false;
                    mainCosts[cEntry.r].cost = cost;
                }

                if (cEntry.label != cLabel) {
                    cCosts = costs.at(cEntry.label);
                    cLabel = cEntry.label;
                }

                auto& c = cCosts[cEntry.r];

                if (c.merge) {
                    // Is not a leaf
                    if (c.prev.label != 0) {
                        q.emplace_back(cEntry.r, c.prev.label, 0, cEntry.maxCost);
                        auto inverse = cEntry.label ^c.prev.label;
                        q.emplace_back(cEntry.r, inverse, 0, cEntry.maxCost);
                    }
                } else {
                    auto n2 = c.prev.node;
                    // TODO: Theoretically we could store the transition cost, would avoid nb lookup...
                    auto cn = instance.getGraph()->nb[cEntry.r][n2];
                    cEntry.cCost += cn;
                    cEntry.maxCost = max(cEntry.maxCost, cEntry.cCost);

                    if (isValid) {
                        state[n2].tp = min(state[n2].tp, cEntry.maxCost);
                        maxCost = max(maxCost, cEntry.maxCost);
                    }
                    state[n2].occurance += 1;

                    q.emplace_back(n2, cEntry.label, cEntry.cCost, cEntry.maxCost);
                }
            }

            return maxCost;
        }
    };
};
#endif //STEINER_HYBRIDSOLVER_H
