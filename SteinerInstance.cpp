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
}

unordered_set<node_id>::iterator SteinerInstance::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    return g_->contractEdge(target, remove, result);
}

void SteinerInstance::removeEdge(node_id u, node_id v) {
    g_->removeEdge(u, v);
}

node_id SteinerInstance::removeNode_(node_id u) {
    if (u < nTerminals) {
        nTerminals--;
        if (u < nTerminals) { // Could be equal now...
            g_->switchVertices(u, nTerminals);
            u = nTerminals;
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
    if (n == *u)
        return g_->removeNode(u);
    else
        return g_->removeNode(n);
}

bool SteinerInstance::addEdge(node_id u, node_id v, cost_id c) {
    g_->addEdge(u, v, c);
}

NodeWithCost* SteinerInstance::getClosestTerminals(node_id v) {
    if (closest_terminals_ == nullptr) {
        // Find distances from terminals to other nodes.
        for(node_id t=0; t < nTerminals; t++)
            g_->findDistances(t);

        // Now calculate the closest terminals
        closest_terminals_ = new NodeWithCost *[g_->getMaxNode()];

        for (int n = 0; n < g_->getMaxNode(); n++) {
            closest_terminals_[n] = new NodeWithCost[nTerminals];

            for(node_id t=0; t < nTerminals; t++) {
                closest_terminals_[n][t].node = t;
                closest_terminals_[n][t].cost = g_->getDistances()[t][n];
            }
            std::sort(closest_terminals_[n], closest_terminals_[n] + nTerminals, greater<NodeWithCost>());
        }
    }
    return closest_terminals_[v];
}


