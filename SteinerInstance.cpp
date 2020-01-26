//
// Created by andre on 25.01.20.
//

#include "SteinerInstance.h"

using namespace steiner;

SteinerInstance::SteinerInstance(Graph *g, unordered_set<node_id> *terminals) : g_(g) {
    for (auto t: *terminals) {
        terminals_.insert(g->getNodeMapping(t));
    }

    // Find distances from terminals to other nodes.
    for (auto t: terminals_) {
        g->findDistances(t);
    }
    // Now calculate the closest terminals
    closest_terminals_ = new NodeWithCost *[g->getNumNodes()];

    for (int n = 0; n < g->getNumNodes(); n++) {
        closest_terminals_[n] = new NodeWithCost[terminals->size()];

        int i = 0;
        for (auto t: terminals_) {
            closest_terminals_[n][i].node = t;
            closest_terminals_[n][i].cost = g->getDistances()[t][n];
            i++;
        }
        std::sort(closest_terminals_[n], closest_terminals_[n] + terminals->size(), greater<NodeWithCost>());
    }
}

void SteinerInstance::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    g_->contractEdge(target, remove, result);
}

void SteinerInstance::removeEdge(node_id u, node_id v) {
    g_->removeEdge(u, v);
}

void SteinerInstance::removeNode(node_id u) {
    g_->removeNode(u);
}

bool SteinerInstance::addEdge(node_id u, node_id v, cost_id c) {
    g_->addEdge(u, v, c);
}