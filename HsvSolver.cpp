//
// Created by aschidler on 1/22/20.
//

#include "HsvSolver.h"

using namespace std;
using namespace boost;

steiner::HsvSolver::HsvSolver(SteinerInstance* instance) : instance_(instance) {
    instance->requestDistanceState(SteinerInstance::exact);
    costs_ = new unordered_map<dynamic_bitset<>, CostInfo>[instance->getGraph()->getMaxNode()];
    store_ = new HashSetLabelStore(instance_->getNumTerminals() - 1, instance->getGraph()->getMaxNode());

    // TODO: Make this configurable?
    if (DualAscent::hasRun) {
        root_ = DualAscent::bestRoot;
    } else {
        root_ = instance_->getNumTerminals() - 1;
    }

    // Move root to the back
    nTerminals_ = instance_->getNumTerminals() - 1;
    instance_->getGraph()->switchVertices(root_, nTerminals_);
    root_ = nTerminals_;

    for(auto u: *instance_->getGraph()->getNodes()) {
        for(auto v: instance_->getGraph()-> nb[u]) {
            if (instance_->getGraph()->nb[v.first][u] != v.second)
                cout << "Error " << u << " " << v.first << endl;
        }
    }

    heuristic_ = new MstHeuristic(instance, root_, nTerminals_);
    //heuristic_ = new DualAscentHeuristic(instance, root_, nTerminals_, instance_->getGraph()->getMaxNode());
}

SteinerTree* steiner::HsvSolver::solve() {
    // Special case, only root
    if (nTerminals_ == 0) {
        auto result = new SteinerTree(root_);
        return result;
    }

    for(int t=0; t < nTerminals_; t++) {
        auto label = dynamic_bitset<>(nTerminals_);
        label.set(t);
        auto entry = QueueEntry(0, t, label);
        auto pred = Predecessor();
        pred.label = nullptr;
        costs_[t].emplace(label, CostInfo(0, pred, true));
        queue_.push(entry);
    }

    while (not queue_.empty()) {
        auto entry = queue_.top();
        queue_.pop();

        auto cost = costs_[entry.node][entry.label].cost;
        if (entry.node == root_ ) {
            if(entry.label.all()) {
                cout << cost << endl;
                return backTrack();
            }
        }
        store_->addLabel(entry.node, &entry.label);
        process_neighbors(entry.node, &entry.label, cost);
        process_labels(entry.node, &entry.label, cost);
    }

    return nullptr;
}

void steiner::HsvSolver::process_neighbors(node_id n, const dynamic_bitset<>* label, cost_id cost) {
    // TODO: Are these getter calls expensive? Maybe retrieve graph once..
    for (auto nb: instance_->getGraph()->nb[n]) {
        auto newCost = cost + nb.second;
        // TODO: Maybe do not copy label all the time?

        auto nbc = costs_[nb.first].find(*label);
        if (nbc == costs_[nb.first].end() || nbc->second.cost > newCost) {
            if (! prune(n, newCost, label)) {
                if (nbc == costs_[nb.first].end()) {
                    auto pred = Predecessor();
                    pred.node = n;
                    costs_[nb.first].emplace(std::piecewise_construct, std::forward_as_tuple(*label), std::forward_as_tuple(newCost, pred, false));
                } else {
                    nbc->second.cost = newCost;
                    nbc->second.prev.node = n;
                    nbc->second.merge = false;
                }

                queue_.emplace(newCost + heuristic_->calculate( nb.first, label), nb.first, *label);
            }
        }
    }
}
void steiner::HsvSolver::process_labels(node_id n, const dynamic_bitset<>* label, cost_id cost) {
    auto other_set = store_->findLabels(n, label);
    for (; other_set->hasNext(); ++(*other_set)) {
        auto combined = *label | **other_set;
        auto newCost = cost + costs_[n][**other_set].cost;

        auto nbc = costs_[n].find(combined);
        if (nbc == costs_[n].end() || nbc->second.cost > newCost) {
            if (! prune(n, newCost, label, &(**other_set), &combined)) {
                if (nbc == costs_[n].end()) {
                    auto pred = Predecessor();
                    pred.label = &(**other_set);
                    costs_[n].emplace(std::piecewise_construct, std::forward_as_tuple(combined), std::forward_as_tuple(newCost, pred, true));
                } else {
                    nbc->second.merge = true;
                    nbc->second.cost = newCost;
                    nbc->second.prev.label = &(**other_set);
                }

                queue_.emplace(newCost + heuristic_->calculate(n, &combined), n, combined);
            }
        }
    }
    delete other_set;
}

bool HsvSolver::prune(node_id n, cost_id cost, const dynamic_bitset<> *label) {
    auto result = pruneBoundCache.find(*label);
    if (result != pruneBoundCache.end()) {
        if (cost > result->second.cost)
            return true;
    }

    prune_check_bound(n, cost, label);

    return false;
}

bool HsvSolver::prune(node_id n, cost_id cost, const dynamic_bitset<> *label1, const dynamic_bitset<>* label2,
                      dynamic_bitset<> *combined) {
    auto result = pruneBoundCache.find(*combined);
    if (result != pruneBoundCache.end()) {
        if (cost > result->second.cost)
            return true;
    }
    if (prune_combine(label1, label2, combined) < cost)
        return true;

    prune_check_bound(n, cost, combined);
    return false;
}

void HsvSolver::prune_check_bound(node_id n, cost_id cost, const dynamic_bitset<> *label) {
    // find minimum distance between n and any terminal not in the label (including root)
    auto dist_c = MAXCOST;
    auto dist_t = 0;

    // Distance to terminals outside the label
    // Since we know there is at least the root outside
    auto closest = instance_->getClosestTerminals(n);
    while (true) {
        if (closest->node == root_ || !(label->test(closest->node))) {
            if (dist_c > closest->cost) {
                dist_c = closest->cost;
                dist_t = closest->node;
            }
            break;
        }
        closest++;
    }

    // Check if we have a cached entry
    auto result = pruneDistCache.find(*label);
    // If yes, just check distances to vertex
    if (result != pruneDistCache.end()) {
        if (result->second.cost < dist_c) {
            dist_c = result->second.cost;
            dist_t = result->second.terminal;
        }
    // Otherwise calculate the distance between terminals in the label and not in the label (includes root)
    } else {
        PruneDistEntry entry(MAXCOST, 0);

        for(int t=0; t < nTerminals_; t++){
            // Terminal is in the label, root guaranteed to not be...
            if (label->test(t)) {
                auto closest2 = instance_->getClosestTerminals(t);
                while (true) {
                    if (closest2->node == root_ || !(label->test(closest2->node))) {
                        if (entry.cost > closest2->cost) {
                            entry.cost = closest2->cost;
                            entry.terminal = closest2->node;
                        }
                        break;
                    }
                    closest2++;
                }
            }
        }
        // Cache value
        pruneDistCache.emplace(*label, entry);

        // Check if better
        if (entry.cost < dist_c) {
            dist_c = entry.cost;
            dist_t = entry.terminal;
        }
    }

    // Store in cache
    auto existing = pruneBoundCache.find(*label);
    if (existing == pruneBoundCache.end()) {
        auto newBs = dynamic_bitset<>(nTerminals_ + 1); // +1 as this may include the root
        newBs.set(dist_t);
        pruneBoundCache.emplace(std::piecewise_construct, std::forward_as_tuple(*label), forward_as_tuple(dist_c + cost, newBs));
    } else {
        existing->second.cost = dist_c + cost;
        existing->second.label.reset();
        existing->second.label.set(dist_t);
    }
}

unsigned int HsvSolver::prune_combine(const dynamic_bitset<> *label1, const dynamic_bitset<> *label2, dynamic_bitset<> *combined) {
    auto result1 = pruneBoundCache.find(*label1);
    if (result1 == pruneBoundCache.end())
        return MAXCOST;
    auto result2 = pruneBoundCache.find(*label2);
    if (result2 == pruneBoundCache.end())
        return MAXCOST;

    auto l1 = *label1; // Add room for root
    auto l2 = *label2; // Add room for root
    l1.push_back(false);
    l2.push_back(false);
    // At least one set must be disjoint...
    if ((l1 & result2->second.label).any() && (l2 & result1->second.label).any())
        return MAXCOST;

    auto cost = result1->second.cost + result2->second.cost;
    auto s = (result1->second.label | result2->second.label) & ~(l1 | l2);
    pruneBoundCache.emplace(std::piecewise_construct, std::forward_as_tuple(*combined), std::forward_as_tuple(cost, s));

    return cost;
}

SteinerTree *HsvSolver::backTrack() {
    auto result = new SteinerTree(root_);
    auto fullLabel = dynamic_bitset<>(nTerminals_);
    fullLabel.flip();
    backTrackSub(root_, &fullLabel, result);

    return result;
}

void HsvSolver::backTrackSub(node_id n, const dynamic_bitset<>* label, SteinerTree* result) {
    auto c = costs_[n].find(*label)->second;
    if (c.merge) {
        // Found a leaf
        if (c.prev.label == nullptr)
            return;

        backTrackSub(n, c.prev.label, result);
        auto inverse = *label ^ *c.prev.label;
        backTrackSub(n, &inverse, result);
    } else {
        auto n2 = c.prev.node;
        auto cn = instance_->getGraph()->nb[n][n2];
        result->addEdge(instance_->getGraph()->getReverseMapping(n), instance_->getGraph()->getReverseMapping(n2), cn);
        backTrackSub(n2, label, result);
    }
}