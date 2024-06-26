//
// Created on 1/22/20.
//

#include "Graph.h"
#include "Structures/Queue.h"

using namespace steiner;

//TODO: Create iterator over edges? Return only if u < v for undirected.
bool steiner::Graph::addMappedEdge(node_id u, node_id v, cost_id cost) {
    if (u == v) // Self loops cannot be optimal
        return false;

    auto un = addMappedNode(u);
    auto vn = addMappedNode(v);

    if (nb[un].count(vn) == 0) {
        this->nb[un].emplace(vn, cost);
        this->nb[vn].emplace(un, cost);
        originalNumEdges_++;
        return true;
    } else if (nb[un][vn] > cost) {
        this->nb[un][vn] = cost;
        this->nb[vn][un] = cost;
        originalNumEdges_++;
        return true;
    }

    return false;
}

bool steiner::Graph::addEdge(node_id u, node_id v, cost_id cost) {
    if (u == v) // Self loops cannot be optimal
        return false;

    if (nodes_.insert(u).second) {
        nodeMap_[u] = u;
        nodeReverseMap_[u] = u;
    }
    if (nodes_.insert(v).second){
        nodeMap_[v] = v;
        nodeReverseMap_[v] = v;
    }

    node_id maxId = max(u, v);
    if (maxId >= nb.size()) {
        nb.resize(maxId+1);
    }
    if (nb[u].count(v) == 0) {
        this->nb[u].emplace(v, cost);
        this->nb[v].emplace(u, cost);
        originalNumEdges_++;
        return true;
    } else if (nb[u][v] > cost) {
        this->nb[u][v] = cost;
        this->nb[v][u] = cost;
        originalNumEdges_++;
        return true;
    }

    return false;
}

node_id steiner::Graph::getNodeMapping(node_id externalId) {
    return nodeMap_[externalId];
}

node_id steiner::Graph::addMappedNode(node_id u) {
    auto result = nodeMap_.find(u);

    if (result == nodeMap_.end()) {
        nb.emplace_back();
        nodeMap_.insert(pair<node_id, node_id>(u, nb.size() - 1));
        nodeReverseMap_.insert(pair<node_id, node_id>(nb.size() - 1, u));
        nodes_.insert(nb.size() - 1);
        changed_ = true;
        return nb.size() - 1;
    }

    return result->second;
}

void steiner::Graph::findDistances(node_id u, cost_id ub) {
    // Init distances
    if (distances_ == nullptr) {
        distances_ = new cost_id*[getMaxNode()];
        distanceInit_ = getMaxNode();
        for(size_t i=0; i < getMaxNode(); i++) {
            distances_[i] = nullptr;
        }
    }
    if (distances_[u] == nullptr) {
        distances_[u] = new cost_id[getMaxNode()];
    }

    for(size_t i=0; i < getMaxNode(); i++) {
        distances_[u][i] = MAXCOST;
    }

    // We could initialize with other known distances...
    // Dijkstra
    auto q = Queue<NodeWithCost>(ub);

    q.emplace(0, u, 0);
    distances_[u][u] = 0;

    while(not q.empty()) {
        auto elem = q.dequeue();

        // already visited...
        if(elem.cost > distances_[u][elem.node])
            continue;

        maxKnownDistance_ = max(maxKnownDistance_, elem.cost);

        for (const auto& v: nb[elem.node]) {
            if (distances_[u][v.first] > elem.cost + v.second) {
                distances_[u][v.first] = elem.cost + v.second;
                q.emplace(elem.cost + v.second, v.first, elem.cost + v.second);
            }
        }
    }
}

set<node_id>::iterator steiner::Graph::removeNode(node_id u) {
    for(auto elem: nb[u]) {
        nb[elem.first].erase(u);
    }
    nb[u].clear();
    auto it = nodes_.find(u);
    if (it != nodes_.end())
        return nodes_.erase(it);
    changed_ = true;
    return it;
}

set<node_id>::iterator steiner::Graph::removeNode(set<node_id>::iterator u) {
    for(const auto& elem: nb[*u]) {
        nb[elem.first].erase(*u);
    }
    nb[*u].clear();
    changed_ = true;
    return nodes_.erase(u);
}

void Graph::removeEdge(node_id u, node_id v) {
    nb[u].erase(v);
    nb[v].erase(u);

    if (nb[u].empty() && nodes_.size() > 1)
        nodes_.erase(u);
    if (nb[v].empty() && nodes_.size() > 1)
        nodes_.erase(v);

    changed_ = true;
}


Graph::EdgeIterator Graph::removeEdge(Graph::EdgeIterator it) {
    auto u = *it.nodeState;
    auto v = it.nbState->first;
    assert(u != v);
    // Remove v first as the iterator points to u, these changes do not influence it.
    nb[v].erase(u);
    if (nb[v].empty() && nodes_.size() > 1)
        nodes_.erase(v);

    // Now remove the same for u, but take care of the iterators!
    it.nbState = nb[u].erase(it.nbState);
    if (nb[u].empty() && nodes_.size() > 1) {
        it.nodeState = nodes_.erase(it.nodeState);
        if (it.nodeState != nodes_.end())
            it.nbState = nb[*it.nodeState].begin();
    }
    it.findNext();
    changed_ = true;
    return it;
}


set<node_id>::iterator Graph::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    vector<Edge> ret;
    for(const auto& n: nb[remove]) {
        if (n.first != target) {
            if (addEdge(target, n.first, n.second) && result != nullptr) {
                // TODO: This mapping is too tightly coupled with the reductions
                result->emplace_back(getReverseMapping(remove), getReverseMapping(target), getReverseMapping(n.first), n.second);
            }
        }
    }

    changed_ = true;
    return removeNode(remove);
}

void Graph::switchVertices(node_id n1, node_id n2) {
    if (n1 == n2)
        return;

    auto on1 = nodeReverseMap_[n1];
    auto on2 = nodeReverseMap_[n2];
    nodeMap_[on1] = n2;
    nodeMap_[on2] = n1;
    nodeReverseMap_[n1] = on2;
    nodeReverseMap_[n2] = on1;

    // Relabel edges
    for(const auto& v: nb[n1]) {
        auto n1c = nb[v.first][n1];
        // Has edges to both nodes, simply switch values
        if (nb[v.first].count(n2) > 0) {
            nb[v.first][n1] = nb[v.first][n2];
            nb[v.first][n2] = n1c;
        } else {
            nb[v.first].erase(n1);
            nb[v.first].emplace(n2, n1c);
        }
    }

    for(const auto& v: nb[n2]) {
        if (v.first == n2) { // In this case n1 was replaced by n2 above
            auto n2c = nb[n1][n2];
            nb[n1].erase(n2);
            nb[n1].emplace(n1, n2c);
        }
        // Other case handled above
        else if (nb[v.first].count(n1) == 0) { // Other case handled above
            auto n2c = nb[v.first][n2];
            nb[v.first].erase(n2);
            nb[v.first].emplace(n1, n2c);
        }
    }

    swap(nb[n1], nb[n2]);
}

node_id Graph::getReverseMapping(node_id internal) {
    return nodeReverseMap_[internal];
}

bool Graph::checkConnectedness(node_id nodeLimit, bool clean) {
    bool seen[getMaxNode()];
    for (node_id i=0; i < getMaxNode(); i++)
        seen[i] = false;

    auto q = vector<node_id>();
    auto nodeIt = nodes_.begin();
    auto first = *nodeIt;
    if (nodeLimit > 0) {
        while(first >= nodeLimit && nodeIt != nodes_.end()) {
            first = *nodeIt;
            ++nodeIt;
        }
    }
    node_id cnt = 1;
    seen[first] = true;
    q.emplace_back(first);

    while(! q.empty()) {
        auto u = q.back();
        q.pop_back();

        for(const auto& v : nb[u]) {
            if (! seen[v.first]) {
                seen[v.first] = true;
                cnt++;
                q.emplace_back(v.first);
            }
        }
    }

    bool connected = (cnt == getNumNodes());

    if (nodeLimit > 0) {
        node_id i=0;
        for(; i < nodeLimit && seen[i]; i++) {
        }
        connected = (i == nodeLimit);

        if (connected && clean) {
            auto it = nodes_.begin();
            while(it != nodes_.end()) {
                if (! seen[*it]) {
                    it = removeNode(it);
                } else {
                    ++it;
                }
            }
        }
    }

    return connected;
}

Graph *Graph::mst() {
    auto result = new Graph(getMaxNode());
    cost_id result_cost = 0;

    // Calculate mst of distance graph
    cost_id minEdgeVal[getMaxNode()];
    bool taken[getMaxNode()];
    node_id minEdgeNode[getMaxNode()];

    cost_id val;
    int idx = -1;

    for(size_t i=0; i < getMaxNode(); i++) {
        minEdgeVal[i] = MAXCOST;
        taken[i] = false;
    }

    // Init
    minEdgeVal[*nodes_.begin()] = 0;
    result->addUnmappedNode(*nodes_.begin());

    for(int i=0; i < getNumNodes(); i++) {
        val = MAXCOST;
        for(const auto k: nodes_) {
            if (minEdgeVal[k] < val) {
                val = minEdgeVal[k];
                idx = k;
            }
        }

        taken[idx] = true;
        if (i > 0)
            result->addEdge(idx, minEdgeNode[idx], minEdgeVal[idx]);
        result_cost += minEdgeVal[idx];
        minEdgeVal[idx] = MAXCOST;

        for (auto& b: nb[idx]) {
            if (! taken[b.first] && b.second < minEdgeVal[b.first]) {
                minEdgeVal[b.first] = b.second;
                minEdgeNode[b.first] = idx;
            }
        }
    }

    maxKnownDistance_ = result_cost;
    return result;
}

cost_id Graph::mst_sum() {
    cost_id result = 0;

    // Calculate mst of distance graph
    cost_id minEdgeVal[getMaxNode()];
    bool taken[getMaxNode()];

    cost_id val;
    int idx = -1;

    for(size_t i=0; i < getMaxNode(); i++) {
        minEdgeVal[i] = MAXCOST;
        taken[i] = false;
    }

    // Init
    minEdgeVal[*nodes_.begin()] = 0;

    for(int i=0; i < getNumNodes(); i++) {
        val = MAXCOST;
        for(const auto& k: nodes_) {
            if (minEdgeVal[k] < val) {
                val = minEdgeVal[k];
                idx = k;
            }
        }

        taken[idx] = true;
        result += minEdgeVal[idx];
        minEdgeVal[idx] = MAXCOST;

        for (auto& b: nb[idx]) {
            if (! taken[b.first] && b.second < minEdgeVal[b.first]) {
                minEdgeVal[b.first] = b.second;
            }
        }
    }

    maxKnownDistance_ = result;
    return result;
}



void Graph::discardDistances() {
    if (distances_ != nullptr) {
        for (size_t i = 0; i < distanceInit_; i++) {
            delete[] distances_[i];
            distances_[i] = nullptr;
        }
        delete[] distances_;
        distances_ = nullptr;
    }
}

vector<node_id> Graph::findPath(node_id u, node_id v) {
    cost_id dist[getMaxNode()];
    node_id p[getMaxNode()];
    for(size_t i=0; i < getMaxNode(); i++) {
        dist[i] = MAXCOST;
    }

    // Dijkstra
    auto q = Queue<NodeWithCost>(0);

    q.emplace(0, u, 0);
    dist[u] = 0;

    bool found = false;
    while(not q.empty()) {
        auto elem = q.dequeue();

        if (elem.node == v) {
            found = true;
            break;
        }

        // already visited...
        if(elem.cost > dist[elem.node])
            continue;

        for (auto& n: nb[elem.node]) {
            if (dist[n.first] > elem.cost + n.second) {
                dist[n.first] = elem.cost + n.second;
                p[n.first] = elem.node;
                q.emplace(elem.cost + n.second, n.first, elem.cost + n.second);
            }
        }
    }

    assert(found);

    vector<node_id> path;
    path.push_back(v);
    while(path.back() != u) {
        path.push_back(p[path.back()]);
    }
    return path;
}

bool Graph::shrink() {
    if (nodes_.size() == nb.size())
        return false;

    node_id last_idx = 0;
    // Goal shrink to nodes_.size()
    auto n = nodes_.begin();
    while(n != nodes_.end()) {
        if (*n >= nodes_.size()) {
            for(; last_idx < nodes_.size() && !nb[last_idx].empty(); last_idx++);
            if (last_idx != *n) { // This can happen if we fully reduced the graph
                switchVertices(*n, last_idx);
                n = nodes_.erase(n);
                nodes_.insert(last_idx);
                last_idx++;
            }
        } else {
            ++n;
        }
    }
    // Remove unnecessary vectors, decreases maxnode
    nb.resize(nodes_.size());
    return true;
}

bool Graph::adaptWeight(node_id up, node_id vp, cost_id original, cost_id modified) {
    auto u = min(up, vp);
    auto v = max(up, vp);

    if (nb.size() > v) {
        auto n = nb[u].find(v);
        if (n != nb[u].end() && n->second == modified) {
            n->second = original;
            nb[v][u] = original;
            return true;
        }
    }

    return false;
}

void Graph::remap(Graph& g) {
    // Find maximum node for sizing
    node_id maxNode = 0;
    for(auto n: nodes_) {
        assert(g.nodeReverseMap_.count(n) > 0);
        maxNode = max(maxNode, g.nodeReverseMap_[n]);
    }
    vector<unordered_map<node_id, cost_id, NodeIdHash>> newNb;
    newNb.resize(maxNode + 1);
    set<node_id> newNodes;

    // Copy datastructures mapped
    for(auto n: nodes_) {
        auto newN = g.nodeReverseMap_[n];
        newNodes.insert(newN);
        for(auto& b: nb[n]) {
            newNb[newN].emplace(g.nodeReverseMap_[b.first], b.second);
        }
    }

    swap(nb, newNb);
    swap(nodes_, newNodes);
}
