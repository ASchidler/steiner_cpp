//
// Created by aschidler on 1/22/20.
//

#include "Graph.h"
using namespace steiner;

//TODO: Create iterator over edges? Return only if u < v for undirected.
bool steiner::Graph::addMappedEdge(node_id u, node_id v, cost_id cost) {
    if (u == v) // Self loops cannot be optimal
        return false;

    auto un = addNode(u);
    auto vn = addNode(v);

    if (nb[un].count(vn) == 0) {
        this->nb[un].emplace(vn, cost);
        this->nb[vn].emplace(un, cost);
        return true;
    } else if (nb[un][vn] > cost) {
        this->nb[un][vn] = cost;
        this->nb[vn][un] = cost;
        return true;
    }

    return false;
}

bool steiner::Graph::addEdge(node_id u, node_id v, cost_id cost) {
    if (u == v) // Self loops cannot be optimal
        return false;
    nodes_.insert(u);
    nodes_.insert(v);
    if (nb[u].count(v) == 0) {
        this->nb[u].emplace(v, cost);
        this->nb[v].emplace(u, cost);
        return true;
    } else if (nb[u][v] > cost) {
        this->nb[u][v] = cost;
        this->nb[v][u] = cost;
        return true;
    }

    return false;
}

node_id steiner::Graph::getNodeMapping(node_id externalId) {
    return nodeMap_[externalId];
}

node_id steiner::Graph::addNode(node_id u) {
    auto result = nodeMap_.find(u);

    if (result == nodeMap_.end()) {
        nb.emplace_back();
        nodeMap_.insert(pair<node_id, node_id>(u, nb.size() - 1));
        nodeReverseMap_.insert(pair<node_id, node_id>(nb.size() - 1, u));
        nodes_.insert(nb.size() - 1);
        return nb.size() - 1;
    }

    return result->second;
}

void steiner::Graph::findDistances(node_id u) {
    // Init distances
    if (distances_ == nullptr) {
        distances_ = new cost_id*[getMaxNode()];
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
    auto q = priority_queue<NodeWithCost>();

    q.emplace(u, 0);
    distances_[u][u] = 0;
//TODO: Implement bucket queue
    while(not q.empty()) {
        auto elem = q.top();
        q.pop();

        // already visited...
        if(elem.cost > distances_[u][elem.node])
            continue;

        for (const auto& v: nb[elem.node]) {
            if (distances_[u][v.first] > elem.cost + v.second) {
                distances_[u][v.first] = elem.cost + v.second;
                q.emplace(v.first, elem.cost + v.second);
            }
        }
    }
}

unordered_set<node_id>::iterator steiner::Graph::removeNode(node_id u) {
    for(auto elem: nb[u]) {
        nb[elem.first].erase(u);
    }
    nb[u].clear();
    auto it = nodes_.find(u);
    if (it != nodes_.end())
        return nodes_.erase(it);
    return it;
}

unordered_set<node_id>::iterator steiner::Graph::removeNode(unordered_set<node_id>::iterator u) {
    for(const auto& elem: nb[*u]) {
        nb[elem.first].erase(*u);
    }
    nb[*u].clear();
    return nodes_.erase(u);
}

void Graph::removeEdge(node_id u, node_id v) {
    nb[u].erase(v);
    nb[v].erase(u);

    if (nb[u].empty())
        nodes_.erase(u);
    if (nb[v].empty())
        nodes_.erase(v);
}


Graph::EdgeIterator Graph::removeEdge(Graph::EdgeIterator it) {
    auto u = *it.nodeState;
    auto v = it.nbState->first;
    assert(u != v);
    // Remove v first as the iterator points to u, these changes do not influence it.
    nb[v].erase(u);
    if (nb[v].empty())
        nodes_.erase(v);

    // Now remove the same for u, but take care of the iterators!
    it.nbState = nb[u].erase(it.nbState);
    if (nb[u].empty()) {
        it.nodeState = nodes_.erase(it.nodeState);
        if (it.nodeState != nodes_.end())
            it.nbState = nb[*it.nodeState].begin();
    }
    it.findNext();
    return it;
}


unordered_set<node_id>::iterator Graph::contractEdge(node_id target, node_id remove, vector<ContractedEdge>* result) {
    vector<Edge> ret;
    for(const auto& n: nb[remove]) {
        if (n.first != target) {
            if (addEdge(target, n.first, n.second) && result != nullptr) {
                result->emplace_back(remove, target, n.first, n.second);
            }
        }
    }

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
        minEdgeVal[idx] = MAXCOST;

        for(const auto k: nodes_) {
            if (! taken[k]) {
                auto dist = nb[idx].find(k);
                if (dist != nb[idx].end() && dist->second < minEdgeVal[k]) {
                    minEdgeVal[k] = dist->second;
                    minEdgeNode[k] = idx;
                }
            }
        }
    }

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

        for(const auto& k: nodes_) {
            if (! taken[k]) {
                auto dist = nb[idx].find(k);
                if (dist != nb[idx].end() && dist->second < minEdgeVal[k]) {
                    minEdgeVal[k] = dist->second;
                }
            }
        }
    }

    return result;
}



void Graph::discardDistances() {
    if (distances_ != nullptr) {
        for (size_t i = 0; i < sizeof(distances_) / sizeof(cost_id); i++) {
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
    auto q = priority_queue<NodeWithCost>();

    q.emplace(u, 0);
    dist[u] = 0;

    while(not q.empty()) {
        auto elem = q.top();
        q.pop();

        if (elem.node == v)
            break;

        // already visited...
        if(elem.cost > dist[elem.node])
            continue;

        for (auto& n: nb[elem.node]) {
            if (dist[n.first] > elem.cost + n.second) {
                dist[n.first] = elem.cost + n.second;
                p[n.first] = elem.node;
                q.emplace(n.first, elem.cost + n.second);
            }
        }
    }

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
                assert(checkConnectedness(0, false));
            }
        } else {
            ++n;
        }
    }
    // Remove unnecessary vectors, decreases maxnode
    while(nb.size() > nodes_.size()) {
        assert(nb.back().empty());
        nb.pop_back();
    }

    return true;
}
