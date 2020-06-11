//
// Created on 1/22/20.
//

#include "HsvSolver.h"
#include <chrono>

using namespace std;
using namespace boost;
using namespace chrono;

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
steiner::HsvSolver<T, T2>::HsvSolver(SteinerInstance* instance, node_id dualAscentLimit) :
        instance_(instance), queue_(instance_->getApproximation().getLowest()), dualAscentLimit_(dualAscentLimit) {
    costs_ = new unordered_map<dynamic_bitset<>, CostInfo>[instance->getGraph()->getMaxNode()];
    store_ = new HashSetLabelStore<T>(instance_->getNumTerminals() - 1, instance->getGraph()->getMaxNode());

    // TODO: Make this configurable?
    root_ = instance->getNumTerminals();
    if (DualAscent::hasRun) {
        root_ = DualAscent::bestRoot;
    }
    if (root_ >= instance->getNumTerminals() && ShortestPath::hasRun)
        root_ = ShortestPath::bestRoot;
    if (root_ >= instance->getNumTerminals())
        root_ = instance_->getNumTerminals() - 1;

    // Move root to the back
    nTerminals_ = instance_->getNumTerminals() - 1;
    maxTerminal_ = 0;
    // Do this in a loop as rotating nTerminals + 1 and subtracting one could cause an overflow
    for (auto t=0; t < nTerminals_; t++) {
        maxTerminal_ <<= 1;
        maxTerminal_ |= 1;
    }
    instance_->getGraph()->switchVertices(root_, nTerminals_);
    root_ = nTerminals_;

    for(auto u: instance_->getGraph()->getNodes()) {
        for(auto v: instance_->getGraph()-> nb[u]) {
            if (instance_->getGraph()->nb[v.first][u] != v.second)
                cout << "Error " << u << " " << v.first << endl;
        }
    }

    if (instance->getGraph()->getNumEdges() <= dualAscentLimit_)
        heuristic_ = new DualAscentHeuristic<T>(instance, root_, nTerminals_, instance_->getGraph()->getMaxNode(), maxTerminal_);
    else
        heuristic_ = new MstHeuristic<T>(instance, root_, nTerminals_, maxTerminal_);


    // Initialize distances. Recalculate after reductions. Also because terminals (root) has been resorted
    instance->setDistanceState(SteinerInstance::invalid);
    instance->getClosestTerminals(0);
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
SteinerResult* steiner::HsvSolver<T, T2>::solve() {
    // Special case, only root
    if (nTerminals_ == 0) {
        auto result = new SteinerResult(0, new Graph(), instance_->getGraph()->getReverseMapping(root_));
        return result;
    }

    for(int t=0; t < nTerminals_; t++) {
        T label = 1;
        label <<= t; // Do this in two steps to avoid overflow errors
        auto pred = Predecessor();
        pred.label = nullptr;
        costs_[t].emplace(label, CostInfo(0, pred, true));
        queue_.emplace(0, 0, 0, t, label);
    }

    while (not queue_.empty()) {
        auto entry = queue_.dequeue();

        auto cost = costs_[entry.node][entry.label].cost;
        if (entry.node == root_ ) {
            if(entry.label == maxTerminal_) {
                cout << cost << endl;
                return backTrack();
            }
        }

        if (cost < entry.originalCost) {
            continue;
        }
        // Checking pruning again does not really eliminate cases

        store_->addLabel(entry.node, entry.label);
        process_neighbors(entry.node, entry.label, cost);
        process_labels(entry.node, entry.label, cost);
    }

    return nullptr;
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
void steiner::HsvSolver<T, T2>::process_neighbors(node_id n, const T label, cost_id cost) {
    // TODO: Are these getter calls expensive? Maybe retrieve graph once..
    for (auto nb: instance_->getGraph()->nb[n]) {
        auto newCost = cost + nb.second;
        // TODO: Maybe do not copy label all the time?

        auto nbc = costs_[nb.first].find(label);
        if (nbc == costs_[nb.first].end() || nbc->second.cost > newCost) {
            if (newCost <= instance_->getUpperBound() && ! prune(n, newCost, label)) {
                if (nbc == costs_[nb.first].end()) {
                    auto pred = Predecessor();
                    pred.node = n;
                    costs_[nb.first].emplace(std::piecewise_construct, std::forward_as_tuple(label), std::forward_as_tuple(newCost, pred, false));
                } else {
                    nbc->second.cost = newCost;
                    nbc->second.prev.node = n;
                    nbc->second.merge = false;
                }
                auto newTotal = newCost + heuristic_->calculate( nb.first, label, instance_->getUpperBound());
                if (newTotal <= instance_->getUpperBound())
                    queue_.emplace(newTotal, newTotal, newCost, nb.first, label);
            }
        }
    }
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
void steiner::HsvSolver<T,T2>::process_labels(node_id n, const T label, cost_id cost) {
    auto other_set = store_->findLabels(n, label);
    for (; other_set->hasNext(); ++(*other_set)) {
        auto combined = label | **other_set;
        // TODO: At least store the pointer to the costs with the label
        // TODO: Maybe run a cleanup in between, where entries according to prune are removed from P...
        auto newCost = cost + costs_[n][**other_set].cost;

        auto nbc = costs_[n].find(combined);
        if (nbc == costs_[n].end() || nbc->second.cost > newCost) {
            if (newCost <= instance_->getUpperBound() && ! prune(n, newCost, label, **other_set, combined)) {
                if (nbc == costs_[n].end()) {
                    auto pred = Predecessor();
                    pred.label = **other_set;
                    costs_[n].emplace(std::piecewise_construct, std::forward_as_tuple(combined), std::forward_as_tuple(newCost, pred, true));
                } else {
                    nbc->second.merge = true;
                    nbc->second.cost = newCost;
                    nbc->second.prev.label = **other_set;
                }

                auto newTotal = newCost +  heuristic_->calculate(n, combined, instance_->getUpperBound());
                if (newTotal <= instance_->getUpperBound())
                    queue_.emplace(newTotal, newTotal, newCost, n, combined);
            }
        }
    }
    delete other_set;
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
bool HsvSolver<T, T2>::prune(node_id n, cost_id cost, const T label) {
    auto result = pruneBoundCache.find(label);
    if (result != pruneBoundCache.end()) {
        if (cost > result->second.cost)
            return true;
    }

    prune_check_bound(n, cost, label);

    return false;
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
bool HsvSolver<T, T2>::prune(node_id n, cost_id cost, const T label1, const T label2,
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

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
void HsvSolver<T, T2>::prune_check_bound(node_id n, cost_id cost, const T label) {
    // find minimum distance between n and any terminal not in the label (including root)
    auto dist_c = MAXCOST;
    auto dist_t = 0;

    // Distance to terminals outside the label
    // Since we know there is at least the root outside
    auto closest = instance_->getClosestTerminals(n);
    while (true) {
        T test = 1;
        test <<= closest->node;
        if (closest->node == root_ || (label & test) > 0) {
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
        auto newBs = dynamic_bitset<>(nTerminals_ + 1); // +1 as this may include the root
        newBs.set(dist_t);
        pruneBoundCache.emplace(std::piecewise_construct, std::forward_as_tuple(label), forward_as_tuple(dist_c + cost, newBs));
    } else {
        if (dist_c + cost < existing->second.cost) {
            existing->second.cost = dist_c + cost;
            existing->second.label.reset();
            existing->second.label.set(dist_t);
        }
    }
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
typename unordered_map<T, typename HsvSolver<T, T2>::PruneDistEntry>::iterator HsvSolver<T, T2>::prune_compute_dist(const T label) {
    PruneDistEntry entry(MAXCOST, 0);

    T test = 1;
    for(int t=0; t < nTerminals_; t++){
        // Terminal is in the label, root guaranteed to not be...
        if ((label & test) > 0) {
            auto closest = instance_->getClosestTerminals(t);
            ++closest; // First one is always the terminal with dist 0, which is in the label, so skip
            while (true) {
                if (closest->node == root_ || !(label->test(closest->node))) {
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
    auto result = pruneDistCache.emplace(*label, entry);

    return result.first;
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
cost_id HsvSolver<T, T2>::prune_combine(const T label1, const T label2, T combined) {
    auto result1 = pruneBoundCache.find(label1);
    if (result1 == pruneBoundCache.end())
        return MAXCOST;
    auto result2 = pruneBoundCache.find(label2);
    if (result2 == pruneBoundCache.end())
        return MAXCOST;

    // At least one set must be disjoint...
    if ((label1 & result2->second.label > 0) && (label2 & result1->second.label > 0))
        return MAXCOST;

    // Was this auto s = (result1->second.label | result2->second.label) & ~(l1 | l2);
    auto l = label1 | label2;
    l.push_back(false);
    l.flip();

    auto s = (result1->second.label | result2->second.label);
    s &= l;
    auto cost = result1->second.cost + result2->second.cost;
    pruneBoundCache.emplace(std::piecewise_construct, std::forward_as_tuple(combined), std::forward_as_tuple(cost, s));

    return cost;
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
SteinerResult* HsvSolver<T, T2>::backTrack() {
    auto result = new SteinerResult(0, new Graph(), instance_->getGraph()->getReverseMapping(root_));
    auto fullLabel = maxTerminal_;
    backTrackSub(root_, fullLabel, result);
    result->cost = result->g->getCost();
    return result;
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value>::type* T2>
void HsvSolver<T, T2>::backTrackSub(node_id n, const T label, SteinerResult* result) {
    auto c = costs_[n].find(label)->second;
    if (c.merge) {
        // Found a leaf
        if (c.prev.label == nullptr)
            return;

        backTrackSub(n, c.prev.label, result);
        auto inverse = label ^ c.prev.label;
        backTrackSub(n, inverse, result);
    } else {
        auto n2 = c.prev.node;
        assert(instance_->getGraph()->nb[n].count(n2) > 0);
        assert(instance_->getGraph()->nb[n][n2] == instance_->getGraph()->nb[n2][n]);
        auto cn = instance_->getGraph()->nb[n][n2];
        result->g->addEdge(instance_->getGraph()->getReverseMapping(n), instance_->getGraph()->getReverseMapping(n2), cn);
        backTrackSub(n2, label, result);
    }
}
