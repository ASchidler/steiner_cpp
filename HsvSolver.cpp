//
// Created on 1/22/20.
//

#include "HsvSolver.h"
#include <chrono>

using namespace std;
using namespace boost;
using namespace chrono;

template <typename T>
steiner::HsvSolver<T>::HsvSolver(SteinerInstance* instance, node_id dualAscentLimit) :
        instance_(instance), queue_(instance_->getApproximation().getLowest()), dualAscentLimit_(dualAscentLimit) {
    costs_ = new unordered_map<T, CostInfo>[instance->getGraph()->getMaxNode()];
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

template <typename T>
SteinerResult* steiner::HsvSolver<T>::solve() {
//    if (test) {
//        test = false;
//        for (auto& u : instance_->getGraph()->getNodes()) {
//            instance_->getGraph()->findDistances(u, instance_->getGraph()->getMaxKnownDistance());
//        }
//    }

    // Special case, only root
    if (nTerminals_ == 0) {
        auto result = new SteinerResult(0, new Graph(), instance_->getGraph()->getReverseMapping(root_));
        return result;
    }

    for(int t=0; t < nTerminals_; t++) {
        T label = 1;
        label <<= t; // Do this in two steps to avoid overflow errors
        auto pred = steiner::Predecessor<T>();
        pred.label = 0;
        costs_[t].emplace(label, CostInfo(0, pred, true));
        queue_.emplace(0, 0, 0, t, label);
    }

    while (not queue_.empty()) {
        auto entry = queue_.dequeue();

        auto& cost = costs_[entry.node][entry.label];
        if (entry.node == root_ ) {
            if(entry.label == maxTerminal_) {
                cout << cost.cost << endl;
                return backTrack();
            }
        }

        if (cost.cost < entry.originalCost) {
            continue;
        }
        if (check_sep(entry.label, entry.node, cost.cost, cost)) {
            continue;
        }
        //propagateUb(entry.node, entry.label, cost);
        // Checking pruning again does not really eliminate cases

        store_->addLabel(entry.node, entry.label);
        process_neighbors(entry);
        process_labels(entry);
    }

    return nullptr;
}

template <typename T>
SteinerResult* HsvSolver<T>::backTrack() {
    auto result = new SteinerResult(0, new Graph(), instance_->getGraph()->getReverseMapping(root_));
    auto fullLabel = maxTerminal_;
    backTrackSub(root_, fullLabel, result);
    result->cost = result->g->getCost();
    return result;
}

template <typename T>
void HsvSolver<T>::backTrackSub(node_id n, const T label, SteinerResult* result) {
    auto c = costs_[n].find(label)->second;
    if (c.merge) {
        // Found a leaf
        if (c.prev.label == 0)
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


template class steiner::HsvSolver<uint16_t>;
template class steiner::HsvSolver<uint32_t>;
template class steiner::HsvSolver<uint64_t>;
template class steiner::HsvSolver<uint128_type>;
