//
// Created by andre on 25.01.20.
//

#include "SteinerInstance.h"
#include "Structures/Queue.h"

using namespace steiner;

SteinerInstance::SteinerInstance(Graph *g, vector<node_id> *terminals) : g_(g) {
    // Move terminals to the front
    nTerminals = 0;
    for (auto cT : *terminals) {
        g_->switchVertices(g_->getNodeMapping(cT), nTerminals);
        nTerminals++;
    }
    closest_terminals_ = nullptr;
}

set<node_id>::iterator SteinerInstance::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    node_id oldRemove = remove;
    remove = removeNode_(remove);
    if (remove < nTerminals)
        invalidateTerminals();

    if (target == remove)
        target = oldRemove;

    // target was the pivot element has been swapped so remove can be deleted...
    return g_->contractEdge(target, remove, result);
}

set<node_id>::iterator
SteinerInstance::contractEdge(set<node_id>::iterator target, node_id remove, vector<ContractedEdge> *result) {
    node_id oldRemove = remove;
    remove = removeNode_(remove);

    auto t = *target;
    bool switched = *target == remove;
    if (switched) {
        t = oldRemove;
    }

    // target was the pivot element has been swapped so remove can be deleted...
    auto it = g_->contractEdge(t, remove, result);
    if (switched) {
        return it;
    }

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
        invalidateTerminals();
        if (u < nTerminals) { // Could be equal now...
            // There should be a terminal and terminals must be connected
            assert(!g_->nb[nTerminals].empty());
            assert(g_->getNodes().count(nTerminals) > 0);
            moveTerminal(u, nTerminals);
            u = nTerminals;
        }
    }
    return u;
}

set<node_id>::iterator SteinerInstance::removeNode(node_id u) {
    u = removeNode_(u);
    return g_->removeNode(u);
}
set<node_id>::iterator SteinerInstance::removeNode(set<node_id>::iterator u) {
    auto n = removeNode_(*u);
    set<node_id>::iterator result;
    if (n == *u)
        result = g_->removeNode(u);
    else
        result =  g_->removeNode(n);

    return result;
}

bool SteinerInstance::addEdge(node_id u, node_id v, cost_id c) {
    return g_->addEdge(u, v, c);
}

NodeWithCost* SteinerInstance::getClosestTerminals(node_id v) {
    // TODO: Is this inefficient for just getting the closest terminals over and over in the solver?
    if (distanceState_ == invalid) {
        clearDistance();
    }

    if (closest_terminals_ == nullptr) {
        closestTerminalsInit = g_->getMaxNode();
        if (closest_terminals_ == nullptr) {
            closest_terminals_ = new NodeWithCost*[g_->getMaxNode()];

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
    if (steinerDistanceState_ == invalid || distanceState_ == invalid)
        clearDistance();
    if (terminalSteinerDistances_ == nullptr)
        calculateSteinerDistance();

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
        terminalSteinerDistances_ = new cost_id*[nTerminals];
        for(int i=0; i < nTerminals; i++) {
            terminalSteinerDistances_[i] = new cost_id[nTerminals];
        }
        terminalSteinerDistanceInit_ = nTerminals;
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
    cost_id dist[nTerminals];
    for (node_id t=0; t < nTerminals; t++) {
        // Find maximum edge on shortest path to other terminals
        //TODO: Optimize
        for(node_id tmp=0; tmp < nTerminals; tmp++) {
            dist[tmp] = MAXCOST;
        }
        dist[t] = 0;
        terminalSteinerDistances_[t][t] = 0;

        Queue<DoubleCostEntry> q(g.getMaxKnownDistance());
        q.emplace(0, t, 0, 0);

        while (! q.empty()) {
            auto elem = q.dequeue();

            if (dist[elem.node] < elem.totalCost)
                continue;

            terminalSteinerDistances_[t][elem.node] = elem.edgeCost;

            for (auto &v: mst->nb[elem.node]) {
                cost_id total = v.second + elem.totalCost;
                if (total < dist[v.first]) {
                    dist[v.first] = total;
                    cost_id maxVal = max(elem.edgeCost, v.second);
                    q.emplace(total, v.first, total, maxVal);
                }
            }
        }
    }
    delete mst;
}

cost_id SteinerInstance::getDistance(node_id n1, node_id n2) {
    if (distanceState_ == invalid) {
        clearDistance();
    }

    if (g_->getDistances() == nullptr || g_->getDistances()[n1] == nullptr)
        g_->findDistances(n1, g_->getMaxKnownDistance());

    return g_->getDistances()[n1][n2];
}

void SteinerInstance::moveTerminal(node_id t, node_id target) {
    g_->switchVertices(t, target);
}

void SteinerInstance::invalidateTerminals() {
    clearDistance();
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



