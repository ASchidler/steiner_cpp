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
#include <algorithm>
#include <iterator>
#include <bit>
#include <bitset>

using namespace std;

// TODO: Since all bitsets have the same size, can't we do this more statically?
namespace steiner {
    template<typename T2>
    union Predecessor {
        T2 label;
        node_id node;
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
            QueueEntry(cost_id cost, cost_id originalCost, node_id node, T label) :
                cost(cost), node(node), label(label), originalCost(originalCost) {

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

        SteinerResult* backTrack();
        void backTrackSub(node_id n, const T label, SteinerResult* result);

        unordered_map<T, CostInfo**> known_nodes;

        __attribute__((noinline))
        bool issep(T label, node_id n, bool* seen, const bool* sep) {
            vector<node_id> q;

            // Start from the root
            q.emplace_back(root_);
            seen[root_] = true;
            auto target = nTerminals_ - std::popcount(label);
            if (target == 0)
                return false;

            // Now perform DFS
            while (! q.empty()) {
                auto u = q.back();
                q.pop_back();

                for(const auto& v : instance_->getGraph()->nb[u]) {
                    if (! seen[v.first] && !sep[v.first]) {
                        if (v.first <= nTerminals_ && v.first != n) { // n might not be in the separator, other terminals must be
                            if(--target == 0)
                                return false;
                        }

                        seen[v.first] = true;
                        q.emplace_back(v.first);
                    }
                }
            }

            return true;
        }

        __attribute__((noinline))
        inline bool check_sep(T label, node_id n, cost_id cost, CostInfo& cinfo) {
            // Other terminal is a separator in itself
            // TODO: Maybe set global upper bound once this case occurred
            // With the other pruning, this should only occur for the closest terminal
            if (n <= this->nTerminals_) {
                T mask = 1;
                mask <<= n;
                if ((mask & label) == 0) {
                    return false;
                }
            }

            // Check if list exists
            auto entry = known_nodes.find(label);
            CostInfo** knodes;
            if (entry == known_nodes.end()) {
                knodes = new CostInfo*[instance_->getGraph()->getMaxNode()] {nullptr};
                known_nodes.emplace(label, knodes);
            } else {
                knodes = (*entry).second;
            }

            knodes[n] = &cinfo;

            // Check connectivity
            bool seen[instance_->getGraph()->getMaxNode()] = {};
            bool ignore[instance_->getGraph()->getMaxNode()] = {};

            unordered_map<node_id, cost_id> costs;
            //findTree(n, label, ignore);
            auto max_nodes = twoPathDist(n, label, costs, ignore);
            Queue<NodeWithCost> qu(max_nodes + 1);
            for(size_t i=0; i < instance_->getGraph()->getMaxNode(); i++) {
                if (ignore[i]) {
                    qu.emplace(max_nodes - costs[i], i, max_nodes - costs[i]);
                }
            }

            while(! qu.empty()) {
                auto nc = qu.dequeue();
                if (ignore[nc.node])
                    continue;

                ignore[nc.node] = true;

                for(const auto& v : instance_->getGraph()->nb[nc.node]) {
                    if (! ignore[v.first]) {
                        // TODO: Check for terminal, would immediately make it a separator
                        if (v.second < nc.cost) {
                            qu.emplace(nc.cost-v.second, nc.node, nc.cost-v.second);
                        }
                    }
                }
            }

            for(size_t i=0; i < instance_->getGraph()->getMaxNode(); i++) {
                if(knodes[i] != nullptr && knodes[i]->cost < cost)
                    ignore[i] = true;
            }

            return issep(label, n, seen, ignore);
        }

        // TODO: Copying adjacency into vector may speed up iteration over neighbors
        inline void process_neighbors(QueueEntry& q) {
            // TODO: Are these getter calls expensive? Maybe retrieve graph once..
            for (auto nb: instance_->getGraph()->nb[q.node]) {
                auto newCost = q.originalCost + nb.second;
                // TODO: Maybe do not copy label all the time?

                auto nbc = costs_[nb.first].find(q.label);
                if (nbc == costs_[nb.first].end() || nbc->second.cost > newCost) {
                    if (newCost <= instance_->getUpperBound() && ! prune(q.node, newCost, q.label)) {
                        if (nbc == costs_[nb.first].end()) {
                            auto pred = Predecessor<T>();
                            pred.node = q.node;
                            costs_[nb.first].emplace(std::piecewise_construct, std::forward_as_tuple(q.label), std::forward_as_tuple(newCost, pred, false));
                        } else {
                            nbc->second.cost = newCost;
                            nbc->second.prev.node = q.node;
                            nbc->second.merge = false;
                        }
                        auto newTotal = newCost + heuristic_->calculate( nb.first, q.label, instance_->getUpperBound());
                        if (newTotal <= instance_->getUpperBound())
                            queue_.emplace(newTotal, newTotal, newCost, nb.first, q.label);
                    }
                }
            }
        }

        inline void process_labels(QueueEntry& q) {
            auto other_set = store_->findLabels(q.node, q.label);
            for (; other_set->hasNext(); ++(*other_set)) {
                auto combined = q.label | **other_set;
                auto& otherEntry = costs_[q.node][**other_set];
                auto newCost = q.originalCost + otherEntry.cost;

                auto nbc = costs_[q.node].find(combined);
                if (nbc == costs_[q.node].end() || nbc->second.cost > newCost) {
                    if (newCost <= instance_->getUpperBound() && ! prune(q.node, newCost, q.label, **other_set, combined)) {
                        if (nbc == costs_[q.node].end()) {
                            auto pred = Predecessor<T>();
                            pred.label = **other_set;
                            costs_[q.node].emplace(std::piecewise_construct, std::forward_as_tuple(combined), std::forward_as_tuple(newCost, pred, true));
                        } else {
                            nbc->second.merge = true;
                            nbc->second.cost = newCost;
                            nbc->second.prev.label = **other_set;
                        }

                        auto newTotal = newCost +  heuristic_->calculate(q.node, combined, instance_->getUpperBound());
                        if (newTotal <= instance_->getUpperBound())
                            queue_.emplace(newTotal, newTotal, newCost, q.node, combined);
                    }
                }
            }
            delete other_set;
        }

        inline bool prune(node_id n, cost_id cost, const T label) {
            auto result = pruneBoundCache.find(label);
            if (result != pruneBoundCache.end()) {
                if (cost > result->second.cost)
                    return true;
            }

            prune_check_bound(n, cost, label);

            return false;
        }

        inline bool prune(node_id n, cost_id cost, const T label1, const T label2,
                                 T combined){
            auto result = pruneBoundCache.find(combined);
            if (result != pruneBoundCache.end()) {
                if (cost > result->second.cost)
                    return true;
            }
            if (prune_combine(label1, label2, combined) < cost)
                return true;

            prune_check_bound(n, cost, combined);
            return false;
        }

        inline void prune_check_bound(node_id n, cost_id cost, const T label) {
            // find minimum distance between n and any terminal not in the label (including root)
            auto dist_c = MAXCOST;
            node_id dist_t = 0;

            // Distance to terminals outside the label
            // Since we know there is at least the root outside
            auto closest = instance_->getClosestTerminals(n);
            while (true) {
                T test = 1;
                test <<= closest->node;
                if (closest->node == root_ || (label & test) == 0) {
                    if (dist_c > closest->cost) {
                        dist_c = closest->cost;
                        dist_t = closest->node;
                    }
                    break;
                }
                ++closest;
                assert(closest <= instance_->getClosestTerminals(n) + nTerminals_);
            }

            // Check if we have a cached entry, otherwise compute it
            auto result = pruneDistCache.find(label);
            // If yes, just check distances to vertex
            if (result == pruneDistCache.end())
                result = prune_compute_dist(label);

            if (result->second.cost < dist_c) {
                dist_c = result->second.cost;
                dist_t = result->second.terminal;
            }

            // Store in cache
            auto existing = pruneBoundCache.find(label);
            if (existing == pruneBoundCache.end()) {
                T newBs = 1;
                newBs <<= dist_t;
                pruneBoundCache.emplace(std::piecewise_construct, std::forward_as_tuple(label), forward_as_tuple(dist_c + cost, newBs));
            } else {
                if (dist_c + cost < existing->second.cost) {
                    existing->second.cost = dist_c + cost;
                    existing->second.label = 1;
                    existing->second.label <<= dist_t;
                }
            }
        }

        inline typename unordered_map<T, typename HsvSolver<T>::PruneDistEntry>::iterator prune_compute_dist(const T label) {
            PruneDistEntry entry(MAXCOST, 0);

            T test = 1;
            for(int t=0; t < nTerminals_; t++){
                // Terminal is in the label, root guaranteed to not be...
                if ((label & test) > 0) {
                    auto closest = instance_->getClosestTerminals(t);
                    ++closest; // First one is always the terminal with dist 0, which is in the label, so skip
                    while (true) {
                        T test2 = 1;
                        test2 <<= closest->node;
                        if (closest->node == root_ || (label & test2) == 0) {
                            if (entry.cost > closest->cost) {
                                entry.cost = closest->cost;
                                entry.terminal = closest->node;
                            }
                            break;
                        }
                        ++closest;
                        assert(closest <= instance_->getClosestTerminals(t) + nTerminals_);
                    }
                }
                test <<= 1;
            }

            // Cache value
            auto result = pruneDistCache.emplace(label, entry);

            return result.first;
        }

        inline cost_id prune_combine(const T label1, const T label2, T combined) {
            auto result1 = pruneBoundCache.find(label1);
            if (result1 == pruneBoundCache.end())
                return MAXCOST;
            auto result2 = pruneBoundCache.find(label2);
            if (result2 == pruneBoundCache.end())
                return MAXCOST;

            // At least one set must be disjoint...
            if ((label1 & result2->second.label) > 0 && (label2 & result1->second.label) > 0)
                return MAXCOST;

            // Was this auto s = (result1->second.label | result2->second.label) & ~(l1 | l2);
            T l = 1;
            l <<= nTerminals_; // Set bit for root
            // Add negated
            l |= (label1 | label2) ^ maxTerminal_;

            T s = (result1->second.label | result2->second.label);
            s &= l;
            auto cost = result1->second.cost + result2->second.cost;
            pruneBoundCache.emplace(std::piecewise_construct, std::forward_as_tuple(combined), std::forward_as_tuple(cost, s));

            return cost;
        }

        struct TwoPathEntry {
            TwoPathEntry(node_id r, T label, cost_id cCost, cost_id maxCost) : r(r), label(label), cCost(cCost), maxCost(maxCost) {}
            const node_id r;
            const T label;
            cost_id cCost;
            cost_id maxCost;
        };

        inline cost_id twoPathDist(node_id r, const T label, unordered_map<node_id, cost_id>& costs, bool* vertices) {
            cost_id maxCost = 0;
            costs.emplace(r, 0u);
            vector<TwoPathEntry> q;
            q.emplace_back(r, label, 0, 0);

            while(! q.empty()) {
                auto cEntry = q.back();
                q.pop_back();

                auto& c = costs_[cEntry.r].find(cEntry.label)->second;

                if (c.merge) {
                    // Is not a leaf
                    if (c.prev.label != 0) {
                        q.emplace_back(cEntry.r, c.prev.label, 0, cEntry.maxCost);
                        auto inverse = cEntry.label ^c.prev.label;
                        q.emplace_back(cEntry.r, inverse, 0, cEntry.maxCost);
                    }
                } else {
                    auto n2 = c.prev.node;
                    auto cn = instance_->getGraph()->nb[cEntry.r][n2];
                    cEntry.cCost += cn;
                    cEntry.maxCost = max(cEntry.maxCost, cEntry.cCost);
                    costs.emplace(n2, cEntry.maxCost);
                    maxCost = max(maxCost, cEntry.maxCost);
                    vertices[n2] = true;

                    q.emplace_back(n2, cEntry.label, cEntry.cCost, cEntry.maxCost);
                }
            }

            return maxCost;
        }

        bool nonDistance = true;
        inline void propagateUb(const node_id r, const T label, const cost_id costs) {
            if (nonDistance) {
                nonDistance = false;
                for(auto u: instance_->getGraph()->getNodes()) {
                    instance_->getGraph()->findDistances(u, instance_->getGraph()->getMaxKnownDistance());
                }
            }

            vector<pair<node_id, T>> q;
            q.emplace_back(r, label);
            auto dists = instance_->getGraph()->getDistances()[r];
            auto pred = steiner::Predecessor<T>();
            pred.label = 0;

            while(! q.empty()) {
                auto cEntry = q.back();
                q.pop_back();

                auto& c = costs_[cEntry.first].find(cEntry.second)->second;

                if (c.merge) {
                    // Is not a leaf
                    if (c.prev.label != 0) {
                        q.emplace_back(cEntry.first, c.prev.label);
                        auto inverse = cEntry.second ^c.prev.label;
                        q.emplace_back(cEntry.first, inverse);
                    }
                } else {
                    auto n2 = c.prev.node;
                    // Using dists is not even necessary, as we could just sum the edges in the queue
                    auto cn = costs + dists[n2];

                    auto nbc = costs_[n2].find(cEntry.second);
                    if (nbc == costs_[n2].end()) {
                        costs_[n2].emplace(std::piecewise_construct, std::forward_as_tuple(cEntry.second), std::forward_as_tuple(cn, pred, true));
                    } else if (cn < nbc->second.cost) {
                        nbc->second.merge = true;
                        nbc->second.cost = cn;
                        nbc->second.prev.label = 0;
                    }

                    q.emplace_back(n2, cEntry.second);
                }
            }
        }

        inline void findTree(const node_id r, const T label, bool* vertices) {
            vector<pair<node_id, T>> q;
            q.emplace_back(r, label);
            //vertices[r] = true;

            while(! q.empty()) {
                auto cEntry = q.back();
                q.pop_back();

                auto& c = costs_[cEntry.first].find(cEntry.second)->second;

                if (c.merge) {
                    // Is not a leaf
                    if (c.prev.label != 0) {
                        q.emplace_back(cEntry.first, c.prev.label);
                        auto inverse = cEntry.second ^c.prev.label;
                        q.emplace_back(cEntry.first, inverse);
                    }
                } else {
                    auto n2 = c.prev.node;
                    vertices[n2] = true;

                    q.emplace_back(n2, cEntry.second);
                }
            }
        }
    };


}

#endif //STEINER_HSVSOLVER_H
