//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"

void steiner::Graph::addEdge(unsigned int u, unsigned int v, unsigned int cost) {
    auto un = addVertex(u);
    auto vn = addVertex(v);
    this->nb[un].emplace_back(vn, cost);
    this->nb[vn].emplace_back(un, cost);
}

unsigned int steiner::Graph::getNodeMapping(unsigned int externalId) {
    return nodeMap_.find(externalId)->second;
}

unsigned int steiner::Graph::addVertex(unsigned int u) {
    auto result = nodeMap_.find(u);

    if (result == nodeMap_.end()) {
        nodes_.push_back(u);
        nb.emplace_back();
        nodeMap_.insert(pair<unsigned int, unsigned int>(u, maxNodeId_));
        maxNodeId_++;
        return maxNodeId_ - 1;
    }

    return result->second;
}