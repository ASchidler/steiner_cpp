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

            static bool comparePtr(CostInfo* a, CostInfo* b) { return (a->cost < b->cost); }
            static bool comparePairPtr(pair<const node_id, CostInfo>* a, pair<const node_id, CostInfo>* b) {
                return a->second.cost < b->second.cost;
            }
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

        void propagate(SteinerInstance& instance, Queue<NodeWithCost>& pq, cost_id ub, unordered_map<node_id, CostInfo>& costs) {
            while(! pq.empty()) {
                auto pqe = pq.dequeue();

                // This might be expensive...
                if (costs.at(pqe.node).cost < pqe.cost)
                    continue;

                // Expand
                for (auto& v : instance.getGraph()->nb[pqe.node]) {
                    cost_id nc = pqe.cost + v.second;
// TODO: Other pruning goes here
// TODO: Check lower bound for bound transgression?
                    if (nc <= ub) {
                        auto it = costs.find(v.first);
                        if (it == costs.end()) {
                            auto pred = Predecessor<T>();
                            pred.node = pqe.node;
                            costs.emplace(std::piecewise_construct, std::forward_as_tuple(v.first), std::forward_as_tuple(nc, pred, false));
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
        }

        void merge(unordered_set<T>& labels, T label, unordered_map<T, unordered_map<node_id, CostInfo>>& costs, Queue<QueueEntry>& q, cost_id ub) {
            auto& ecosts = costs.at(label);
            for(T otherLabel: labels) {
                if ((label & otherLabel) == 0) {
                    T newLabel = label | otherLabel;
                    if (labels.find(newLabel) != labels.end())
                        continue;

                    Queue<NodeWithCost> sq(ub + 1);
                    auto& ocost = costs.at(otherLabel);
                    auto& ncost = costs.emplace(newLabel, unordered_map<node_id, CostInfo>()).first->second;

                    // merge costs
                    cost_id minCost = MAXCOST;
                    for(auto& ce: ecosts) {
                        auto oe = ocost.find(ce.first);
                        if (oe != ocost.end()) {
                            cost_id nc = ce.second.cost + oe->second.cost;
                            if (nc <= ub) {
                                minCost = min(minCost, nc);

                                // Check if other
                                auto nit = ncost.find(ce.first);
                                if (nit == ecosts.end()) {
                                    auto pred = Predecessor<T>();
                                    pred.label = otherLabel;
                                    ncost.emplace(std::piecewise_construct, std::forward_as_tuple(ce.first), std::forward_as_tuple(nc, pred, true));
                                    sq.emplace(nc, ce.first, nc);
                                } else if (nit->second.cost > nc) {
                                    nit->second.cost = nc;
                                    nit->second.prev.label = otherLabel;
                                    nit->second.merge = true;
                                    sq.emplace(nc, ce.first, nc);
                                }
                            }
                        }
                    }
                    if (!sq.empty()) {
                        q.emplace(minCost, newLabel, minCost);
                    }
                }
            }
        }

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

                // Propagate
                Queue<NodeWithCost> sq(ub + 1);
                sq.emplace(0, t, 0);
                propagate(instance, sq, ub, ce.first->second);

                // Increment
                targetLabel <<= 1u;
            }
            // TODO: Cost table for labels to check if re-evaluation is necessary
            targetLabel -= 1; // Target label now contains the label for all terminals

            while(! q.empty()) {
                auto qe = q.dequeue();
                auto& ecosts = costs.at(qe.label);

                if (labels.find(qe.label) != labels.end())
                    continue;

                // TODO: if label in labels, with consistency we can (maybe) skip, otherwise we need a revision number
                labels.emplace(qe.label);
                // At this point label is complete

                // Found root
                if(qe.label == targetLabel) {
                    cout << ecosts.begin()->second.cost << endl;
                    break;
                }

                Queue<NodeWithCost> sq(ub+1);
                for(auto& cc: ecosts) {
                    sq.emplace(cc.second.cost, cc.first, cc.second.cost);
                }
                propagate(instance, sq, ub, ecosts);

                // Sort costs
                auto sz = ecosts.size();
                pair<const node_id, CostInfo>* localCosts[sz];
                pair<const node_id, CostInfo>** costIdx = localCosts;

                for(auto& ce: ecosts) {
                    *costIdx = &ce;
                    costIdx++;
                }
                std::sort(localCosts, localCosts + sz, CostInfo::comparePairPtr);

                // Compare
                // TODO: Maybe use one struct array instead of 4 arrays?
                cost_id limits[instance.getGraph()->getMaxNode()];
                std::fill_n(limits, instance.getGraph()->getMaxNode(), MAXCOST);
                cost_id tp[instance.getGraph()->getMaxNode()];
                std::fill_n(tp, instance.getGraph()->getMaxNode(), MAXCOST);
                node_id vertices[instance.getGraph()->getMaxNode()] = {};
                cost_id maxC = 0;

                for(node_id i=0; i < sz; i++) {
                    auto& cc = *localCosts[i];
                    maxC = max(maxC, twoPathDist(cc.first, qe.label, vertices, costs, cc.second.cost, instance, limits, tp));
                }

                char sep[instance.getGraph()->getMaxNode()] = {};
                Queue<NodeWithCost> qu(maxC);

                // Add all nodes present in all trees
                for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (vertices[i] == sz) {
                        qu.emplace(maxC - tp[i], i, maxC - tp[i]);
                        sep[i] = 1;
                    }
                }

                // Find separator candidate (N) based on two path
                while(! qu.empty()) {
                    auto nc = qu.dequeue();
                    if (sep[nc.node])
                        continue;

                    sep[nc.node] = 1;

                    for(const auto& v : instance.getGraph()->nb[nc.node]) {
                        if (! sep[v.first]) {
                            if (v.second < nc.cost) {
                                qu.emplace(nc.cost-v.second, nc.node, nc.cost-v.second);
                            }
                        }
                    }
                }

                // Extend the candidate, until it is a separator
                vector<pair<cost_id, node_id>> updatedCosts;
                for (node_id i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    updatedCosts.emplace_back(limits[i], i);
                }
                std::sort(updatedCosts.begin(), updatedCosts.end(), std::greater());

                vector<node_id> cq;
                node_id target = instance.getNumTerminals() - std::popcount(qe.label);
                cost_id overallLimit = MAXCOST;
                bool ran = false;
                for(auto& cse: updatedCosts) {
                    if (target == 0)
                        break;

                    // Part of the separator
                    if (sep[cse.second] == 1)
                        continue;

                    overallLimit = cse.first;

                    sep[cse.second] = 2;
                    if (ran) {
                        for(const auto& v : instance.getGraph()->nb[cse.second]) {
                            if (sep[v.first] == 2)
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
                            if (sep[v.first] == 2) {
                                if (v.first < instance.getNumTerminals()) {
                                    T mask = 1; // This construct avoids an overflow if 128 bits are used
                                    mask <<= v.first;
                                    if((mask & qe.label) == 0)
                                        target--;
                                }

                                sep[v.first] = 3;
                                cq.emplace_back(v.first);
                            }
                        }
                    }
                }

                auto it = ecosts.begin();
                while(it != ecosts.end()) {
                    if (it->second.cost > overallLimit || it->second.cost > limits[it->first])
                        it = ecosts.erase(it);
                    else
                        ++it;
                }

                // Merge
                merge(labels, qe.label, costs, q, ub);
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

        inline cost_id twoPathDist(node_id r, const T label, node_id* vertices, unordered_map<T,
                unordered_map<node_id, CostInfo>>& costs, cost_id cost, SteinerInstance& instance, cost_id* limits,
                cost_id* tp) {
            cost_id maxCost = 0;

            vertices[r] += 1;

            //costs.emplace(r, 0u);
            vector<TwoPathEntry> q;
            q.emplace_back(r, label, 0, 0);

            unordered_map<node_id, CostInfo>* cCosts;
            auto cLabel = 0;

            bool isValid = limits[r] >= cost;
            if (isValid)
                tp[r] = 0;

            while(! q.empty()) {
                auto cEntry = q.back();
                q.pop_back();
                limits[cEntry.r] = min(cost, limits[cEntry.r]);

                if (cEntry.label != cLabel) {
                    cCosts = &(costs.at(cEntry.label));
                    cLabel = cEntry.label;
                }

                auto& c = cCosts->at(cEntry.r);

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
                        tp[n2] = min(tp[n2], cEntry.maxCost);
                        maxCost = max(maxCost, cEntry.maxCost);
                    }
                    vertices[n2] += 1;

                    q.emplace_back(n2, cEntry.label, cEntry.cCost, cEntry.maxCost);
                }
            }

            return maxCost;
        }
    };
};
#endif //STEINER_HYBRIDSOLVER_H
