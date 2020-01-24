//
// Created by aschidler on 1/22/20.
//

#include "HsvSolver.h"

using namespace std;
using namespace boost;

steiner::HsvSolver::HsvSolver(SteinerInstance* instance) : instance_(instance) {
    costs_ = new unordered_map<dynamic_bitset<>, CostInfo>[instance->getGraph()->getNumNodes()];
    store_ = new HashSetLabelStore(instance_->getTerminals()->size() - 1, instance->getGraph()->getNumNodes());

    // TODO: Make this configurable?
    if (DualAscent::hasRun) {
        root_ = DualAscent::bestRoot;
    } else {
        root_ = *instance_->getTerminals()->begin();
    }

    int i = 0;
    for(auto t:*instance->getTerminals()) {
        if (t != root_) {
            tmap_.insert(pair<node_id, node_id>(t, i));
            terminals_.insert(t);
            i++;
        }
    }
    nTerminals_ = terminals_.size();
    //heuristic_ = new MstHeuristic(instance, &tmap_, &terminals_, root_);
    heuristic_ = new DualAscentHeuristic(instance, &tmap_, &terminals_, root_);
}

steiner::Graph* steiner::HsvSolver::solver() {
    // Special case, only root
    if (terminals_.empty()) {
        auto result = new Graph();
        result->addNode(root_);
        return result;
    }

    for(const auto& elem: this->terminals_) {
        auto label = dynamic_bitset<>(nTerminals_);
        label.set(tmap_[elem]);
        auto entry = QueueEntry(0, elem, label);
        auto pred = Predecessor();
        pred.label = nullptr;
        costs_[elem].insert(pair<dynamic_bitset<>, CostInfo>(label, CostInfo(0, pred, true)));
        queue_.push(entry);
    }

    while (not queue_.empty()) {
        auto entry = queue_.top();
        queue_.pop();

        auto cost = costs_[entry.node].find(entry.label)->second.cost;
        if (entry.node == root_ ) {
            if((~entry.label).none()) {
                cout << cost << endl;
                break;
            }
        }
        store_->addLabel(entry.node, &entry.label);
        process_neighbors(entry.node, &entry.label, cost);
        process_labels(entry.node, &entry.label, cost);
    }

    return new Graph();
}

void steiner::HsvSolver::process_neighbors(node_id n, dynamic_bitset<>* label, cost_id cost) {
    // TODO: Are these getter calls expensive? Maybe retrieve graph once..
    for (auto nb: instance_->getGraph()->nb[n]) {
        auto newCost = cost + nb.second;
        // TODO: Maybe do not copy label all the time? If labels are stored centrally once created, pointers could be
        // used. Thay would also simplify hashing. But how to find labels in a central store?, use a central HashSet?
        // Could be put into the central label store, as each label goes through there...

        auto nbc = costs_[nb.first].find(*label);
        if (nbc == costs_[nb.first].end() || nbc->second.cost > newCost) {
            if (! prune(n, newCost, label)) {
                if (nbc == costs_[nb.first].end()) {
                    auto pred = Predecessor();
                    pred.node = n;
                    costs_[nb.first].insert(pair<dynamic_bitset<>, CostInfo>(*label, CostInfo(newCost, pred, false)));
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
void steiner::HsvSolver::process_labels(node_id n, dynamic_bitset<>* label, cost_id cost) {
    auto other_set = store_->findLabels(n, label);
    for (; other_set->hasNext(); ++(*other_set)) {
        auto combined = *label | **other_set;
        auto newCost = cost + costs_[n].find(**other_set)->second.cost;

        auto nbc = costs_[n].find(combined);
        if (nbc == costs_[n].end() || nbc->second.cost > newCost) {
            if (! prune(n, newCost, label, &(**other_set), &combined)) {
                if (nbc == costs_[n].end()) {
                    auto pred = Predecessor();
                    pred.label = &(**other_set);
                    costs_[n].insert(pair<dynamic_bitset<>, CostInfo>(combined, CostInfo(newCost, pred, false)));
                } else {
                    nbc->second.cost = newCost;
                    nbc->second.prev.label = &(**other_set);
                    nbc->second.merge = true;
                }

                queue_.emplace(newCost + heuristic_->calculate(n, &combined), n, combined);
            }
        }
    }
    delete other_set;
}

bool HsvSolver::prune(node_id n, cost_id cost, dynamic_bitset<> *label) {
    // TODO: Another label copy...
    auto result = pruneBoundCache.find(*label);
    if (result != pruneBoundCache.end()) {
        if (cost > result->second.cost)
            return true;
    }

    prune_check_bound(n, cost, label);

    return false;
}

bool HsvSolver::prune(node_id n, cost_id cost, dynamic_bitset<> *label1, const dynamic_bitset<>* label2,
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

void HsvSolver::prune_check_bound(node_id n, cost_id cost, dynamic_bitset<> *label) {
    // find minimum distance between n and any terminal not in the label (including root)
    auto dist_c = instance_->getGraph()->getDistances()[root_][n];
    auto dist_t = root_;

    // distance to terminals outside the label
    auto closest = instance_->getClosestTerminals(n);
    for (int i=0; i < sizeof(closest)/sizeof(*closest) ;i++) {
        auto elem = instance_->getClosestTerminals(n);
        if (!(label->test(tmap_[elem->node]))) {
            if (dist_c > elem->cost) {
                dist_c = elem->cost;
                dist_t = elem->node;
            }
            break;
        }
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

        // TODO: Precalculate closest terminals...
        for (auto t: terminals_) {
            // Terminal is in the label
            if (label->test(tmap_[t])) {
                auto dist = instance_->getGraph()->getDistances()[t][root_];
                if (dist < entry.cost) {
                    entry.cost = dist;
                    entry.terminal = root_;
                }

                for (int i=0; i < sizeof(closest)/sizeof(*closest) ;i++) {
                    auto elem = instance_->getClosestTerminals(n);
                    if (t != elem->node && !(label->test(tmap_[elem->node]))) {
                        if (entry.cost > elem->cost) {
                            entry.cost = elem->cost;
                            entry.terminal = elem->node;
                        }
                        break;
                    }
                }
            }
        }
        // Cache value
        pruneDistCache.insert(pair<dynamic_bitset<>, PruneDistEntry>(*label, entry));

        // Check if better
        if (entry.cost < dist_c) {
            dist_c = entry.cost;
            dist_t = entry.terminal;
        }
    }

    // Either way, we now have the minimum distance...
    auto existing = pruneBoundCache.find(*label);
    if (existing == pruneBoundCache.end()) {
        PruneBoundEntry newEntry(dist_c + cost, dynamic_bitset<>(nTerminals_));
        newEntry.label.set(tmap_[dist_t]);
        pruneBoundCache.insert(pair<dynamic_bitset<>, PruneBoundEntry>(*label, newEntry));
    } else {
        existing->second.cost = dist_c + cost;
        existing->second.label.reset();
        existing->second.label.set(tmap_[dist_t]);
    }
}

unsigned int HsvSolver::prune_combine(dynamic_bitset<> *label1, const dynamic_bitset<> *label2, dynamic_bitset<> *combined) {
    auto result1 = pruneBoundCache.find(*label1);
    if (result1 == pruneBoundCache.end())
        return UINT_MAX;
    auto result2 = pruneBoundCache.find(*label2);
    if (result2 == pruneBoundCache.end())
        return UINT_MAX;

    // At least one set must be disjoint...
    if ((*label1 & result2->second.label).any() && (*label2 & result1->second.label).any())
        return UINT_MAX;

    // TODO: These many dynamic set creations are probably not very efficient
    auto cost = result1->second.cost + result2->second.cost;
    auto s = (result1->second.label | result2->second.label) & ~(*label1 | *label2);
    PruneBoundEntry entry(cost, s);
    pruneBoundCache.insert(pair<dynamic_bitset<>, PruneBoundEntry>(*combined, entry));

    return cost;
}