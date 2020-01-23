//
// Created by aschidler on 1/22/20.
//

#include "HsvSolver.h"

using namespace std;
using namespace boost;

steiner::HsvSolver::HsvSolver(SteinerInstance* instance) : instance_(instance), store_(instance_->getTerminals()->size(), instance->getGraph()->getNumNodes()) {
    costs_ = new unordered_map<dynamic_bitset<>, CostInfo>[instance->getGraph()->getNumNodes()];

    int i = -1;
    for(auto t:*instance->getTerminals()) {
        if (i == -1) {
            root_ = t;
        }
        else {
            tmap_.insert(pair<unsigned int, unsigned int>(t, i));
            terminals_.insert(t);
        }
        i++;
    }
    nTerminals_ = terminals_.size();
    heuristic_ = new MstHeuristic(instance->getGraph(), &tmap_, &terminals_, root_);
}

steiner::Graph* steiner::HsvSolver::solver() {
    // Special case, only root
    if (terminals_.empty()) {
        auto result = new Graph();
        result->addVertex(root_);
        return result;
    }

    // Find distances from terminals to other nodes. Actually t to t would suffice
    for (auto t: terminals_) {
        instance_->getGraph()->findDistances(t);
    }
    instance_->getGraph()->findDistances(root_);

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
        store_.addLabel(entry.node, &entry.label);
        process_neighbors(entry.node, &entry.label, cost);
        process_labels(entry.node, &entry.label, cost);
    }

    return new Graph();
}

void steiner::HsvSolver::process_neighbors(unsigned int n, dynamic_bitset<>* label, unsigned int cost) {
    // TODO: Are these getter calls expensive? Maybe retrieve graph once..
    for (auto nb: instance_->getGraph()->nb[n]) {
        auto newCost = cost + nb.cost;
        // TODO: Maybe do not copy label all the time? If labels are stored centrally once created, pointers could be
        // used. Thay would also simplify hashing. But how to find labels in a central store?, use a central HashSet?
        // Could be put into the central label store, as each label goes through there...

        auto nbc = costs_[nb.node].find(*label);
        if (nbc == costs_[nb.node].end() || nbc->second.cost > newCost) {
            if (! prune(n, newCost, label)) {
                if (nbc == costs_[nb.node].end()) {
                    auto pred = Predecessor();
                    pred.node = n;
                    costs_[nb.node].insert(pair<dynamic_bitset<>, CostInfo>(*label, CostInfo(newCost, pred, false)));
                } else {
                    nbc->second.cost = newCost;
                    nbc->second.prev.node = n;
                    nbc->second.merge = false;
                }

                // TODO: Prune and heuristic...
                queue_.emplace(newCost + heuristic_->calculate( nb.node, label), nb.node, *label);
            }
        }
    }

}
void steiner::HsvSolver::process_labels(unsigned int n, dynamic_bitset<>* label, unsigned int cost) {
    auto other_set = store_.findLabels(n, label);
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

                // TODO: Prune and heuristic...
                queue_.emplace(newCost + heuristic_->calculate(n, &combined), n, combined);
            }
        }
    }
    delete other_set;
}

bool HsvSolver::prune(unsigned int n, unsigned int cost, dynamic_bitset<> *label) {
    // TODO: Another label copy...
    auto result = pruneBoundCache.find(*label);
    if (result != pruneBoundCache.end()) {
        if (cost > result->second.cost)
            return true;
    }

    prune_check_bound(n, cost, label);

    return false;
}

bool HsvSolver::prune(unsigned int n, unsigned int cost, dynamic_bitset<> *label1, const dynamic_bitset<>* label2,
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

void HsvSolver::prune_check_bound(unsigned int n, unsigned int cost, dynamic_bitset<> *label) {
    unsigned int dist_c = UINT8_MAX;
    unsigned int dist_t = 0;

    // find minimum distance between n and any terminal not in the label (including root)
    auto dist = instance_->getGraph()->getDistances()[root_][n];
    if (dist < dist_c) {
        dist_c = dist;
        dist_t = root_;
    }
    // distance to terminals outside the label
    for (auto t: terminals_) {
        // Terminal outside the label
        if (!(label->test(tmap_[t]))) {
            dist = instance_->getGraph()->getDistances()[t][n];
            if (dist < dist_c) {
                dist_c = dist;
                dist_t = t;
            }
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
        PruneDistEntry entry(UINT_MAX, UINT_MAX);

        // TODO: Precalculate closest terminals...
        for (auto t: terminals_) {
            // Terminal is in the label
            if (label->test(tmap_[t])) {
                auto dist = instance_->getGraph()->getDistances()[t][root_];
                if (dist < entry.cost) {
                    entry.cost = dist;
                    entry.terminal = root_;
                }

                // Compare to all other terminals
                for (auto t2: terminals_) {
                    if (t2 != t && !(label->test(tmap_[t]))) {
                        dist = instance_->getGraph()->getDistances()[t][t2];
                        if (dist < entry.cost) {
                            entry.cost = dist;
                            entry.terminal = t2;
                        }
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