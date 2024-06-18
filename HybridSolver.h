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
    public:
        HybridSolver(SteinerInstance& instance) : costQueue_(instance.getUpperBound()) {

        }

        struct MinimalCostInfo {
            MinimalCostInfo(cost_id cost, MinimalCostInfo* prev, MinimalCostInfo* prev2, node_id node, T label) :
                    cost(cost), prev1(prev), prev2(prev2), node(node), label(label) {}
            MinimalCostInfo() = default;
            cost_id cost = MAXCOST;
            MinimalCostInfo* prev1 = nullptr;
            MinimalCostInfo* prev2 = nullptr;
            node_id node = 0;
            T label = 0;

            static bool compareNode(MinimalCostInfo& a, MinimalCostInfo& b) {
                return a.node < b.node;
            }
            static bool compareCost(MinimalCostInfo& a, MinimalCostInfo& b) {
                return a.cost < b.cost;
            }
            static bool compareCostReverse(MinimalCostInfo& a, MinimalCostInfo& b) {
                return a.cost > b.cost;
            }
        };

        struct CostInfo {
            CostInfo(unsigned int cost, T prev, bool merge) : cost(cost), prev(prev), merge(merge), valid(true) {
            }
            CostInfo() = default;
            cost_id cost = MAXCOST;
            T prev = 0;
            MinimalCostInfo* prev1;
            MinimalCostInfo* prev2;
            bool merge = false;
            bool valid = false;
            bool dummy = false;
            node_id newIdx = 0;

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
            TwoPathEntry(MinimalCostInfo* ref, cost_id cCost, cost_id maxCost) : ref(ref), cCost(cCost), maxCost(maxCost), r(0), label(0) {}
            const node_id r;
            const T label;
            MinimalCostInfo* ref = nullptr;
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
        Queue<NodeWithCost> costQueue_;

    public:

        void propagate(SteinerInstance& instance, cost_id ub, CostInfo* costs) {
            while(! costQueue_.empty()) {
                auto pqe = costQueue_.dequeue();

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
                        costs[v.first].prev1 = nullptr;
                        costs[v.first].prev2 = nullptr;
                        // TODO: This probably causes a cache miss
                        // TODO: Other pruning goes here
                        // TODO: Check lower bound for bound transgression?

                        costs[v.first].dummy = costs[pqe.node].dummy;
                        if (nc > ub) {
                            costs[v.first].cost = ub + 1;
                            costs[v.first].valid = false;
                        } else {
                            costQueue_.emplace(nc, v.first, nc);
                        }
                    }
                }
            }
        }

        void merge(unordered_map<T, vector<MinimalCostInfo>>& labels, T label,
                unordered_map<T, vector<MinimalCostInfo>>& costs,
                Queue<QueueEntry>& q, cost_id ub, SteinerInstance& instance, vector<MinimalCostInfo> ecosts) {

            for(auto& otherLabel: labels) {
                if ((label & otherLabel.first) == 0) {
                    T newLabel = label | otherLabel.first;

                    cost_id minCost = MAXCOST;

                    bool change = false;
                    auto it1 = ecosts.begin();
                    auto it2 = otherLabel.second.begin();
                    // TODO: Lazy init this
                    // TODO: This may not work, if costs are revisited, this needs to be initialized
                    auto& ncost = costs[newLabel];
                    auto it3 = ncost.begin();

                    while(it1 != ecosts.end() && it2 != otherLabel.second.end()) {
                        if (it1->node < it2->node)
                            ++it1;
                        else if (it2->node > it2->node)
                            ++it2;
                        else {
                            cost_id nc = it1->cost + it2->cost;
                            if (nc <= ub) {
                                minCost = min(minCost, nc);
                                change = true;

                                while (it3 != ncost.end() && it3->node < it1->node)
                                    ++it3;

                                if (it3 == ncost.end()) {
                                    ncost.emplace_back(nc, &(*it1), &(*it2), it1->node, newLabel);
                                    it3 = ncost.end(); // May have changed
                                }
                                else if (it1->node == it3->node) {
                                    it3->cost = nc;
                                    it3->prev1 = &(*it1);
                                    it3->prev2 = &(*it2);
                                }
                                else {
                                    // TODO: This could be done more efficiently
                                    it3 = ncost.emplace(it3, nc, &(*it1), &(*it2), it1->node, newLabel);
                                    // We know that the next item examined does not have the same node
                                    ++it3;
                                }
                            }
                            ++it1;
                            ++it2;
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
            unordered_map<T, vector<MinimalCostInfo>> labels;
            unordered_map<T, vector<MinimalCostInfo>> costs;
            cost_id ub = instance.getUpperBound(); // Shorthand
            Queue<QueueEntry> q(ub+1);

            T targetLabel = 1;
            for(node_id t=0; t < instance.getNumTerminals(); t++) {
                // Initialize Queue
                q.emplace(0, targetLabel, 0);
                // Initialize costs
                auto& newCosts = costs[targetLabel];
                newCosts.emplace_back(0, nullptr, nullptr, t, targetLabel);

                // Increment
                targetLabel <<= 1u;
            }

            targetLabel -= 1; // Target label now contains the label for all terminals

            while(! q.empty()) {
                auto qe = q.dequeue();

                auto costit = costs.find(qe.label);
                if (costit == costs.end())
                    continue;
                auto& ecostset = costit->second;
                CostInfo ecosts[instance.getGraph()->getMaxNode()];

                // Found root
                if(qe.label == targetLabel) {
                    cout << ecostset.at(0).cost << endl;
                    cout << "Done" << endl;
                    return;
                }

                // Initialize cost array
                vector<pair<CostInfo*, node_id>> localCosts;
                for(auto& ce: ecostset) {
                    ecosts[ce.node].valid = true;
                    ecosts[ce.node].cost = ce.cost;
                    ecosts[ce.node].prev1 = ce.prev1;
                    ecosts[ce.node].prev2 = ce.prev2;
                    ecosts[ce.node].prev = ce.node;
                    localCosts.emplace_back(ecosts + ce.node, ce.node);
                }
                std::sort(localCosts.begin(), localCosts.end(), CostInfo::comparePairPtr);

                // Find two path distances
                PruneState state[instance.getGraph()->getMaxNode()];
                cost_id maxC = 0;
                node_id cnt = 0;
                for(auto& entry: localCosts) {
                    if (entry.first->valid && !entry.first->dummy) {
                        cnt++;
                        maxC = max(maxC, twoPathDist(entry.second, qe.label, ecosts, entry.first->cost, instance, state));
                    }
                }

                // Propagate costs and invalid flag
                for (node_id i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (ecosts[i].cost <= ub && ecosts[i].valid)
                        costQueue_.emplace(ecosts[i].cost, i, ecosts[i].cost);
                }
                propagate(instance, ub, ecosts);

                propagateTwoPath(state, maxC, cnt, instance);

                // Extend the candidate, until it is a separator
                cost_id overallLimit = findSeparatorBound(instance, qe.label, state, ecosts);

                auto cit = labels.emplace(make_pair(qe.label, vector<MinimalCostInfo>()));
                auto& labelEntry = cit.first->second;
                // Clear pre-existing entries
                if (!cit.second)
                    labelEntry.clear();

                bool anyUnresolved = false;
                for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                    if (ecosts[i].cost <= overallLimit && !ecosts[i].dummy && ecosts[i].valid) {
                        if (ecosts[i].prev1 == nullptr) {
                            // In this case no pointer is yet known. As the previous node can be higher, second pass is required
                            labelEntry.emplace_back(ecosts[i].cost, nullptr, nullptr, i, qe.label);
                            // Use index here, as a pointer may change upon relocation
                            ecosts[i].newIdx = labelEntry.size() - 1;
                            anyUnresolved = true;
                        }
                        else
                            labelEntry.emplace_back(ecosts[i].cost, ecosts[i].prev1, ecosts[i].prev2, i, qe.label);
                    }
                }

                // This sets the predecessor whenever it is in this vector
                if (anyUnresolved) {
                    for(auto& cnEntry: labelEntry) {
                        if (cnEntry.prev1 == nullptr && ecosts[cnEntry.node].prev != cnEntry.node) {
                            auto cnIdx = ecosts[ecosts[cnEntry.node].prev].newIdx;
                            cnEntry.prev1 = &(labelEntry.at(cnIdx));
                        }
                    }
                }

                if (labelEntry.size() > 0) {
                    // Merge
                    std::sort(labelEntry.begin(), labelEntry.end(), MinimalCostInfo::compareNode);
                    merge(labels, qe.label, costs, q, ub, instance, labelEntry);
                } else {
                    labels.erase(qe.label);
                }
                costs.erase(qe.label);
            }

            cout << "No result" << endl;
            exit(3);
        }

        cost_id findSeparatorBound(SteinerInstance& instance, T label, PruneState* state, CostInfo* ecosts) {
            vector<pair<CostInfo*, node_id>> localCosts;
            for(size_t i=0; i < instance.getGraph()->getMaxNode(); i++) {
                localCosts.emplace_back(ecosts + i, i);
            }
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

        cost_id twoPathDist(node_id r, const T label, CostInfo* mainCosts, cost_id cost, SteinerInstance& instance, PruneState* state) {
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
                node_id node;
                node_id prev;
                bool merge;

                if (cEntry.ref == nullptr) {
                    node = cEntry.r;
                    if (mainCosts[cEntry.r].prev1 == nullptr) {
                        if (cEntry.r != mainCosts[cEntry.r].prev)
                            twoPathQueue_.emplace_back(mainCosts[cEntry.r].prev, cEntry.label, cEntry.cCost, cEntry.maxCost);
                        prev = mainCosts[cEntry.r].prev;
                        merge = false;
                    } else {
                        twoPathQueue_.emplace_back(mainCosts[cEntry.r].prev1, 0, cEntry.maxCost);
                        twoPathQueue_.emplace_back(mainCosts[cEntry.r].prev2, 0, cEntry.maxCost);
                        merge = true;
                    }
                } else {
                    node = cEntry.ref->node;
                    if (cEntry.ref->prev2 == nullptr && cEntry.ref->prev1 != nullptr) {
                        twoPathQueue_.emplace_back(cEntry.ref->prev1, cEntry.cCost, cEntry.maxCost);
                        prev = cEntry.ref->prev1->node;
                        merge = false;
                    } else {
                        if (cEntry.ref->prev1 != nullptr) {
                            twoPathQueue_.emplace_back(cEntry.ref->prev1, 0, cEntry.maxCost);
                            twoPathQueue_.emplace_back(cEntry.ref->prev2, 0, cEntry.maxCost);
                        }
                        merge = true;
                    }
                }

                if(mainCosts[node].cost > cost) {
                    mainCosts[node].dummy = true;
                    mainCosts[node].cost = cost;
                }

                if (!merge) {
                    // TODO: Theoretically we could store the transition cost, would avoid nb lookup...
                    // TODO: Actually it is just the diff between the previous and these costs
                    auto cn = instance.getGraph()->nb[cEntry.r][prev];
                    cEntry.cCost += cn;
                    cEntry.maxCost = max(cEntry.maxCost, cEntry.cCost);

                    //TODO: We do not need to emplace, we can change...
                    state[prev].tp = min(state[prev].tp, cEntry.maxCost);
                    state[prev].occurance += 1;

                    maxCost = max(maxCost, cEntry.maxCost);
                }
            }

            return maxCost;
        }
    };
};
#endif //STEINER_HYBRIDSOLVER_H
