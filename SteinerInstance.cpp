//
// Created by andre on 25.01.20.
//

#include "SteinerInstance.h"

using namespace steiner;

SteinerInstance::SteinerInstance(Graph *g, vector<node_id> *terminals) : g_(g) {
    // Move terminals to the front
    nTerminals = 0;
    for (auto cT : *terminals) {
        g_->switchVertices(g_->getNodeMapping(cT), nTerminals);
        nTerminals++;
    }
    closest_terminals_ = nullptr;
    maxTerminals = nTerminals;
}

unordered_set<node_id>::iterator SteinerInstance::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    node_id oldRemove = remove;
    remove = removeNode_(remove);
    if (remove < nTerminals)
        invalidateTerminals();

    if (target == remove)
        target = oldRemove;

    // target was the pivot element has been swapped so remove can be deleted...
    return g_->contractEdge(target, remove, result);
}

unordered_set<node_id>::iterator
SteinerInstance::contractEdge(unordered_set<node_id>::iterator target, node_id remove, vector<ContractedEdge> *result) {
    node_id oldRemove = remove;
    remove = removeNode_(remove);
    if (remove < nTerminals)
        invalidateTerminals();

    auto t = *target;
    if (*target == remove)
        t = oldRemove;

    // target was the pivot element has been swapped so remove can be deleted...
    auto it = g_->contractEdge(t, remove, result);
    if (remove == *target)
        return it;

    return target;
}

void SteinerInstance::removeEdge(node_id u, node_id v) {
    g_->removeEdge(u, v);
    // Terminal deletions must not happen
    assert(u >= nTerminals || !g_->nb[u].empty());
    assert(v >= nTerminals || !g_->nb[v].empty());
}

Graph::EdgeIterator SteinerInstance::removeEdge(Graph::EdgeIterator it) {
    return g_->removeEdge(it);
}

node_id SteinerInstance::removeNode_(node_id u) {
    if (u < nTerminals) {
        nTerminals--;
        if (u < nTerminals) { // Could be equal now...
            // There should be a terminal and terminals must be connected
            assert(!g_->nb[nTerminals].empty());
            assert(g_->getNodes().count(nTerminals) > 0);
            moveTerminal(u, nTerminals);
            u = nTerminals;
            invalidateTerminals();
        }
    }
    return u;
}

unordered_set<node_id>::iterator SteinerInstance::removeNode(node_id u) {
    u = removeNode_(u);
    return g_->removeNode(u);
}
unordered_set<node_id>::iterator SteinerInstance::removeNode(unordered_set<node_id>::iterator u) {
    auto n = removeNode_(*u);
    unordered_set<node_id>::iterator result;
    if (n == *u)
        result = g_->removeNode(u);
    else
        result =  g_->removeNode(n);

    return result;
}

bool SteinerInstance::addEdge(node_id u, node_id v, cost_id c) {
    g_->addEdge(u, v, c);
}

NodeWithCost* SteinerInstance::getClosestTerminals(node_id v) {
    // TODO: Is this inefficient for just getting the closest terminals over and over in the solver?
    if (distanceState_ == invalid)
        clearDistance();
    if (closest_terminals_ == nullptr) {
        if (closest_terminals_ == nullptr) {
            closest_terminals_ = new NodeWithCost *[g_->getMaxNode()];

            for(int n=0; n < g_->getMaxNode(); n++) {
                closest_terminals_[n] = nullptr;
            }
        }

        // Now calculate the closest terminals
        for (auto n: g_->getNodes()) {
            closest_terminals_[n] = new NodeWithCost[nTerminals];

            for(node_id t=0; t < nTerminals; t++) {
                closest_terminals_[n][t].node = t;
                closest_terminals_[n][t].cost = getDistance(t, n);
            }
            std::sort(closest_terminals_[n], closest_terminals_[n] + nTerminals, greater<NodeWithCost>());
        }
    }
    return closest_terminals_[v];
}

/**
 * Computing the exact steiner distances is very costly, so this is only a heuristic value.
 */
cost_id SteinerInstance::getSteinerDistance(node_id u, node_id v) {
    if (terminalSteinerDistances_ == nullptr || steinerDistanceState_ == invalid) {
        clearDistance();
        calculateSteinerDistance();
    }

    cost_id sd = MAXCOST;
    auto edgeCost = g_->nb[u].find(v);
    if (edgeCost != g_->nb[u].end())
        sd = edgeCost->second;

    // TODO: Make this 3 configurable?
    node_id nValues1 = min((node_id) 3, nTerminals);
    node_id nValues2 = nValues1;

    if (u < nTerminals)
        nValues1 = 1;
    if (v < nTerminals)
        nValues2 = 1;

    for(node_id v1=0; v1 < nValues1; v1++) {
        for(node_id v2=0; v2 < nValues2; v2++) {
            auto cl1 = getClosestTerminals(u)[v1];
            auto cl2 = getClosestTerminals(v)[v2];
            auto val = max(cl1.cost, cl2.cost);

            if (cl1.node != cl2.node)
                val = max(val, terminalSteinerDistances_[cl1.node][cl2.node]);

            sd = min(sd, val);
        }
    }

    return sd;
}

void SteinerInstance::calculateSteinerDistance() {
    // Note that the number of terminals never goes up, so the array may be too large, but why care?
    if (terminalSteinerDistances_ == nullptr) {
        terminalSteinerDistances_ = new cost_id*[maxTerminals];
        for(int i=0; i < maxTerminals; i++)
            terminalSteinerDistances_[i] = new cost_id[maxTerminals];
    }
    steinerDistanceState_ = exact;

    // Create distance network
    Graph g = Graph(nTerminals);
    for (node_id t1=0; t1 < nTerminals; t1++) {
        for(node_id t2=t1+1; t2 < nTerminals; t2++) {
            g.addEdge(t1, t2, getDistance(t1, t2));
        }
    }

    // Find shortest path in mst
    auto mst = g.mst();
    bool found[nTerminals];
    cost_id dist[nTerminals];
    found[0] = true;
    for (node_id t=0; t < nTerminals; t++) {
        // Find maximum edge on shortest path to other terminals
        //TODO: Optimize
        for(node_id tmp=0; tmp < nTerminals; tmp++) {
            dist[tmp] = MAXCOST;
        }
        dist[t] = 0;
        terminalSteinerDistances_[t][t] = 0;

        priority_queue<DoubleCostEntry> q;
        q.emplace(t, 0, 0);

        while (! q.empty()) {
            auto elem = q.top();
            q.pop();

            if (dist[elem.node] < elem.totalCost)
                continue;

            terminalSteinerDistances_[t][elem.node] = elem.edgeCost;

            for (auto &v: mst->nb[elem.node]) {
                cost_id total = v.second + elem.totalCost;
                if (total < dist[v.first]) {
                    dist[v.first] = total;
                    cost_id maxVal = max(elem.edgeCost, v.second);
                    q.emplace(v.first, total, maxVal);
                }
            }
        }
//        node_id goal = nTerminals - t - 1;
//        for(node_id tmp=0; tmp < nTerminals; tmp++) {
//            dist[tmp] = MAXCOST;
//            if (tmp > t)
//                found[tmp] = false;
//        }
//        dist[t] = 0;
//        terminalSteinerDistances_[t][t] = 0;
//
//        priority_queue<DoubleCostEntry> q;
//        q.emplace(t, 0, 0);
//
//        while (goal > 0) {
//            auto elem = q.top();
//            q.pop();
//            if (elem.totalCost > dist[elem.node])
//                continue;
//            if (! found[elem.node]) {
//                goal--;
//                found[elem.node] = true;
//            }
//
//            for (auto &v: mst->nb[elem.node]) {
//                cost_id total = v.second + elem.totalCost;
//                if (total < dist[v.first]) {
//                    dist[v.first] = total;
//                    cost_id maxVal = max(elem.edgeCost, v.second);
//
//                    terminalSteinerDistances_[t][elem.node] = maxVal;
//                    terminalSteinerDistances_[elem.node][t] = maxVal;
//
//                    q.emplace(v.first, total, maxVal);
//                }
//            }
//        }
    }
    delete mst;
}

cost_id SteinerInstance::getDistance(node_id n1, node_id n2) {
    if (distanceState_ == invalid) {
        clearDistance();
    }

    if (g_->getDistances() == nullptr || g_->getDistances()[n1] == nullptr)
        g_->findDistances(n1);

    return g_->getDistances()[n1][n2];
}

void SteinerInstance::moveTerminal(node_id t, node_id target) {
    g_->switchVertices(t, target);
}

void SteinerInstance::invalidateTerminals() {
    clearClosest_();
    setSteinerDistanceState(invalid);
}

void SteinerInstance::shrink() {
    g_->checkConnectedness(nTerminals, true);
    if (g_->shrink()) {
        approximationState_ = invalid;
        distanceState_ = invalid;
        steinerDistanceState_ = invalid;
        clearDistance();
    }
}



