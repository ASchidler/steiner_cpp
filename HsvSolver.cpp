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
}

steiner::Graph* steiner::HsvSolver::solver() {
    // Special case, len(terminals) == 1
    // TODO: This returns one node, but no idea which. There has to be a mapping from idx to node in the graph class!
    // TODO: In the graph class use nNodes as upper bound, don't expect that many nodes!
    if (terminals_.empty()) {
        auto result =  new Graph();
        result->addVertex(root_);
        return result;
    }

    for(const auto& elem: this->terminals_) {
        auto label = dynamic_bitset<>(nTerminals_);
        label[tmap_[elem]] = true;
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
            queue_.push(QueueEntry(newCost, nb.node, *label));
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
            queue_.push(QueueEntry(newCost, n, combined));
        }
    }
    delete other_set;
}