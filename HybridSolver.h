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
            CostInfo(unsigned int cost, T prev, bool merge) : cost(cost), prev(prev), merge(merge), valid(true) {
            }
            CostInfo() = default;
            cost_id cost = MAXCOST;
            T prev = 0;
            bool merge = false;
            bool valid = false;
            bool dummy = false;

            static bool comparePtr(CostInfo* a, CostInfo* b) { return (a->cost < b->cost); }

            // TODO: Pointer access my not be aligned
            static bool comparePairPtr(pair<CostInfo*, node_id>& a, pair<CostInfo*, node_id>& b) {
                return a.first->cost < b.first->cost;
            }

            static bool comparePairPtrT(pair<CostInfo*, node_id>& a, pair<CostInfo*, node_id>& b) {
                return a.first->cost > b.first->cost;
            }
        };

        struct MinimalCostInfo {
            MinimalCostInfo(cost_id cost, T prev, bool merge, T label) : cost(cost), prev(prev), merge(merge), label(label) {}

            cost_id cost;
            T prev;
            bool merge;
            T label;
        };

        struct QueueEntry {
            QueueEntry(T label, cost_id estimate) : label(label), estimate(estimate) {

            }

            T label;
            cost_id estimate;

            bool operator<(const QueueEntry& p2) const {
                return estimate > p2.estimate;
            }
            bool operator>(const QueueEntry& p2) const {
                return estimate > p2.estimate;
            }
        };
        struct TwoPathEntry {
            TwoPathEntry(node_id r, T label, cost_id cCost, cost_id maxCost) : r(r), label(label), cCost(cCost), maxCost(maxCost) {}
            const node_id r;
            const T label;
            cost_id cCost;
            cost_id maxCost;
        };

        struct PruneState {
            enum NodeState {nfound, found, naddeed, seperator};
            PruneState() = default;

            cost_id tp = MAXCOST;
            node_id occurance = 0;
            NodeState sep = naddeed;
        };
    private:
        vector<TwoPathEntry> twoPathQueue_;
        vector<node_id> nodeQueue_;

    public:

        void propagate(SteinerInstance& instance, Queue<NodeWithCost>& pq, cost_id ub, CostInfo* costs) {
            while(! pq.empty()) {
                auto pqe = pq.dequeue();

                if (costs[pqe.node].cost < pqe.cost || !costs[pqe.node].valid)
                    continue;

                // Expand
                for (auto& v : instance.getGraph()->nb[pqe.node]) {
                    cost_id nc = pqe.cost + v.second;
                    if (costs[v.first].cost > nc) {
                        costs[v.first].cost = nc;
                        costs[v.first].prev = pqe.node;
                        costs[v.first].merge = false;
                        costs[v.first].valid = true;

                        // TODO: This probably causes a cache miss
                        // TODO: Other pruning goes here
                        // TODO: Check lower bound for bound transgression?

                        costs[v.first].dummy = costs[pqe.node].dummy;
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

        void merge(unordered_map<T, unordered_map<node_id, MinimalCostInfo>>& labels, T label, unordered_map<T, CostInfo*>& costs, Queue<QueueEntry>& q, cost_id ub, SteinerInstance& instance) {
            auto& ecosts = costs.at(label);
            for(auto& otherLabel: labels) {
                if ((label & otherLabel.first) == 0) {
                    T newLabel = label | otherLabel.first;

                    CostInfo* ncost = nullptr;
                    cost_id minCost = MAXCOST;
                    auto it = costs.find(newLabel);
                    if (it != costs.end()) {
                        ncost = it->second;
                    }
                    // merge costs

                    bool change = false;
                    for(auto& oentry: otherLabel.second) {
                        if (ecosts[oentry.first].valid) {
                            cost_id nc = ecosts[oentry.first].cost + oentry.second.cost;
                            if (nc <= ub) {
                                minCost = min(minCost, nc);
                                if (ncost == nullptr) {
                                    ncost = new CostInfo[instance.getGraph()->getMaxNode()];
                                    costs.emplace(newLabel, ncost);
//                                    auto lit = labels.find(newLabel);
//                                    if (lit != labels.end()) {
//                                        for(auto& lentry: lit->second) {
//                                            ncost[lentry.first].valid = true;
//                                            ncost[lentry.first].prev = lentry.second.prev;
//                                            ncost[lentry.first].cost = lentry.second.cost;
//                                            ncost[lentry.first].merge = lentry.second.merge;
//                                        }
//                                    }
                                }

                                if (ncost[oentry.first].cost > nc) {
                                    change = true;
                                    ncost[oentry.first].cost = nc;
                                    ncost[oentry.first].valid = true;
                                    ncost[oentry.first].dummy = false;
                                    ncost[oentry.first].prev = otherLabel.first;
                                    ncost[oentry.first].merge = true;
                                }
                            }
                        }
                    }
                    if (change) {
                        // TODO: In case of consistency, re-adding is probably not necessary
                        q.emplace(std::popcount(newLabel), newLabel, std::popcount(newLabel));
                        //q.emplace(minCost, newLabel, minCost);
                    }
                }
            }
        }

        void solve(SteinerInstance& instance) {
            // TODO: If we shrink the graph at the beginning, we can just iterate over the nodes, without getnodes?
            unordered_map<T, unordered_map<node_id, MinimalCostInfo>> labels;
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
                newCosts[t].prev = 0;

                // Increment
                targetLabel <<= 1u;
            }

            targetLabel -= 1; // Target label now contains the label for all terminals

            while(! q.empty()) {
                auto qe = q.dequeue();

                auto costit = costs.find(qe.label);
                if (costit == costs.end())
                    continue;
                CostInfo* ecosts = costit->second;

                // Found root
                if(qe.label == targetLabel) {
                    while(!ecosts->valid)
                        ecosts++;

                    cout << ecosts->cost << endl;
                    cout << "Done" << endl;
                    return;
                }

                // Sort costs
                vector<pair<CostInfo*, node_id>> localCosts;
                for (node_id i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    localCosts.emplace_back(ecosts + i, i);
                }
                std::sort(localCosts.begin(), localCosts.end(), CostInfo::comparePairPtr);

                PruneState state[instance.getGraph()->getMaxNode()];
                cost_id maxC = 0;
                node_id cnt = 0;
                for(auto& entry: localCosts) {
                    if (entry.first->valid && !entry.first->dummy) {
                        cnt++;
                        maxC = max(maxC, twoPathDist(entry.second, qe.label, ecosts, labels, entry.first->cost, instance, state));
                    }
                }

                // Propagate costs and invalid flag
                Queue<NodeWithCost> sq(ub+1);
                for (node_id i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (ecosts[i].cost <= ub && ecosts[i].valid)
                        sq.emplace(ecosts[i].cost, i, ecosts[i].cost);
                }
                propagate(instance, sq, ub, ecosts);

                propagateTwoPath(state, maxC, cnt, instance);

                // Extend the candidate, until it is a separator
                cost_id overallLimit = findSeparatorBound(localCosts, qe.label, instance, state);

                unordered_map<node_id, MinimalCostInfo>* labelEntry = nullptr;
                for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (ecosts[i].cost <= overallLimit && !ecosts[i].dummy && ecosts[i].valid) {
                        if (labelEntry == nullptr) {
                            auto cit = labels.emplace(make_pair(qe.label, unordered_map<node_id, MinimalCostInfo>()));
                            labelEntry = &(cit.first->second);
                            // Clear pre-existing entries
                            if (!cit.second)
                                labelEntry->clear();
                        }
                        labelEntry->emplace(std::piecewise_construct,
                                            std::forward_as_tuple(i),
                                            std::forward_as_tuple(ecosts[i].cost, ecosts[i].prev, ecosts[i].merge, qe.label));
                    } else {
                        ecosts[i].valid = false;
                    }
                }

                if (labelEntry != nullptr) {
                    // Merge
                    merge(labels, qe.label, costs, q, ub, instance);
                } else {
                    labels.erase(qe.label);
                }
                costs.erase(qe.label);
                delete [] ecosts;
            }

            cout << "No result" << endl;
            exit(3);
        }

        cost_id findSeparatorBound(vector<pair<CostInfo*, node_id>> localCosts, T label, SteinerInstance& instance, PruneState* state) {
            std::sort(localCosts.begin(), localCosts.end(), CostInfo::comparePairPtrT);

            node_id target = instance.getNumTerminals() - std::popcount(label);
            node_id target2 = target;
            cost_id overallLimit = MAXCOST;
            nodeQueue_.clear();
            int cnt2 = 0;

            // First add all nodes, until all the sought after terminals have been added
            auto it = localCosts.begin();
            while(it != localCosts.end() && target > 0) {
                state[it->second].sep = PruneState::nfound;
                overallLimit = it->first->cost;

                if (it->second < instance.getNumTerminals()) {
                    T mask = 1; // This construct avoids an overflow if 128 bits are used
                    mask <<= it->second;
                    if((mask & label) == 0) {
                        target--;
                        if (nodeQueue_.empty())
                            nodeQueue_.emplace_back(it->second);
                    }
                }
                ++it;
            }

            // Now all terminals have been added, we can check if they are connected and add vertices if necessary
            while(it != localCosts.end() && target2 > 0) {
                // Check connectivity
                while (! nodeQueue_.empty() && target2 > 0) {
                    auto cqe = nodeQueue_.back();
                    nodeQueue_.pop_back();

                    if (state[cqe].sep != PruneState::nfound)
                        continue;
                    state[cqe].sep = PruneState::found;

                    if (cqe < instance.getNumTerminals()) {
                        T mask = 1; // This construct avoids an overflow if 128 bits are used
                        mask <<= cqe;
                        if((mask & label) == 0)
                            target2--;
                    }

                    for(const auto& v : instance.getGraph()->nb[cqe]) {
                        if (state[v.first].sep == PruneState::nfound) {
                            nodeQueue_.emplace_back(v.first);
                        }
                    }
                }

                // Connected?
                if (target2 == 0)
                    break;

                // Extend graph
                bool found = false;
                while(it != localCosts.end() && !found) {
                    // If the node is part of the separator, skip
                    if (state[it->second].sep != PruneState::seperator) {
                        // The upper bound is it least the current cost
                        overallLimit = it->first->cost;
                        state[it->second].sep = PruneState::nfound;

                        // Check if any neighbor is connected

                        for (const auto &v : instance.getGraph()->nb[it->second]) {
                            if (state[v.first].sep == PruneState::found) {
                                nodeQueue_.emplace_back(it->second);
                                found = true;
                                break;
                            }
                        }
                    }
                    ++it;
                }
            }

            return overallLimit;
        }

        void propagateTwoPath(PruneState* state, cost_id maxCost, node_id cnt, SteinerInstance& instance) {
            /*  Propagate two path costs, whenever a vertex exists in all trees
                This works a little bit different than usual Dijkstra. The goal here is to mark all vertices within
                range of tp. We therefore take the distance to the maximum cost, as this way the priority queue
                works as intended (otherwise we'd need to reverse prioritization order)
                */
            Queue<NodeWithCost> qu(maxCost);
            for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                if (state[i].occurance == cnt && state[i].tp > 0) {
                    qu.emplace(maxCost - state[i].tp, i, maxCost - state[i].tp);
                }
            }

            // Find separator candidate (N) based on two path
            while(! qu.empty()) {
                auto nc = qu.dequeue();
                if (state[nc.node].sep == PruneState::seperator)
                    continue;

                state[nc.node].sep = PruneState::seperator;

                for(const auto& v : instance.getGraph()->nb[nc.node]) {
                    if (state[v.first].sep == 0) {
                        if (v.second + nc.cost < maxCost) {
                            qu.emplace(nc.cost + v.second, v.first, nc.cost + v.second);
                        }
                    }
                }
            }
        }

        cost_id twoPathDist(node_id r, const T label, CostInfo* mainCosts, unordered_map<T,
                unordered_map<node_id, MinimalCostInfo>>& labels, cost_id cost, SteinerInstance& instance, PruneState* state) {
            cost_id maxCost = 0;

            state[r].occurance += 1;
            twoPathQueue_.emplace_back(r, label, 0, 0);

            //unordered_map<node_id, MinimalCostInfo>* cCosts = nullptr;
            auto cLabel = 0;

            state[r].tp = 0;
            bool merge;
            T prev;

            while(! twoPathQueue_.empty()) {
                auto cEntry = twoPathQueue_.back();
                twoPathQueue_.pop_back();

                if(mainCosts[cEntry.r].cost > cost) {
                    mainCosts[cEntry.r].dummy = true;
                    mainCosts[cEntry.r].cost = cost;
                }

                if (cEntry.label == label) {
                    merge = mainCosts[cEntry.r].merge;
                    prev = mainCosts[cEntry.r].prev;
                } else {
//                    if (cEntry.label != cLabel) {
//                        cCosts = &(labels.at(cEntry.label));
//                        cLabel = cEntry.label;
//                    }
//
//                    auto &c = cCosts->at(cEntry.r);
                    auto &c = labels.at(cEntry.label).at(cEntry.r);
                    merge = c.merge;
                    prev = c.prev;
                }

                if (merge) {
                    // Is not a leaf
                    if (prev != 0) {
                        twoPathQueue_.emplace_back(cEntry.r, prev, 0, cEntry.maxCost);
                        auto inverse = cEntry.label ^ prev;
                        twoPathQueue_.emplace_back(cEntry.r, inverse, 0, cEntry.maxCost);
                    }
                } else {
                    // TODO: Theoretically we could store the transition cost, would avoid nb lookup...
                    // TODO: Actually it is just the diff between the previous and these costs
                    auto cn = instance.getGraph()->nb[cEntry.r][prev];
                    cEntry.cCost += cn;
                    cEntry.maxCost = max(cEntry.maxCost, cEntry.cCost);

                    //TODO: We do not need to emplace, we can change...
                    state[prev].tp = min(state[prev].tp, cEntry.maxCost);
                    state[prev].occurance += 1;

                    maxCost = max(maxCost, cEntry.maxCost);
                    twoPathQueue_.emplace_back(prev, cEntry.label, cEntry.cCost, cEntry.maxCost);
                }
            }

            return maxCost;
        }
    };
};
#endif //STEINER_HYBRIDSOLVER_H
