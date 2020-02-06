//
// Created by andre on 31.01.20.
//

#include "LocalOptimization.h"


void steiner::LocalOptimization::vertexInsertion(Graph* dg, SteinerResult& tr, node_id nTerminals) {
    for(auto n: dg->getNodes()) {
        if (tr.g->getNodes().count(n) == 0) {
            // Find neighbors that are in the solution
            vector<NodeWithCost> shared;
            for(auto& b: dg->nb[n]) {
                if (tr.g->getNodes().count(b.first) > 0) {
                    shared.emplace_back(b.first, b.second);
                }
            }
            // There exist at least two neighbors (1 would not have a chance to succeed)
            if (shared.size() > 1) {
                auto cp = new Graph(*tr.g, false);
                // Add first edge and try to improve
                cp->addEdge(n, shared.back().node, shared.back().cost);
                shared.pop_back();

                // Check all other shared neighbors for improvement
                for(auto b: shared) {
                    // Find shortest path from
                    auto path = cp->findPath(n, b.node);
                    auto maxEdge = Edge(path[0], path[1], tr.g->nb[path[0]][path[1]]);

                    for (size_t cNode = 2; cNode < path.size(); cNode++) {
                        auto c = cp->nb[path[cNode - 1]][path[cNode]];
                        if (c > maxEdge.cost) {
                            maxEdge.cost = c;
                            maxEdge.u = path[cNode - 1];
                            maxEdge.v = path[cNode];
                        }
                    }
                    // If the path cost is higher, using the found edge is cheaper
                    if (maxEdge.cost > b.cost) {
                        // Iterator is on dg, not tr, so it stays valid
                        cp->addEdge(n, b.node, b.cost);
                        cp->removeEdge(maxEdge.u, maxEdge.v);
                    }
                }

                // Remove degree 1 non-terminals
                bool changed = true;
                while (changed) {
                    changed = false;
                    auto nit = cp->getNodes().begin();
                    while(nit != cp->getNodes().end()) {
                        if (cp->nb[*nit].size() == 1 && *nit != tr.root && *nit >= nTerminals) {
                            nit = cp->removeNode(nit);
                            changed = true;
                        }
                        else
                            ++nit;
                    }
                }

                // Check if we made progress:
                auto trCost = cp->getCost();
                if (tr.cost >= trCost) {
                    delete tr.g;
                    tr.g = cp;
                    tr.cost = trCost;
                } else {
                    delete cp;
                }
            }
        }
    }
}

void steiner::LocalOptimization::pathExchange(Graph& g, SteinerResult& tr, node_id numTerminals, bool favorNew) {
    if (tr.g->getNumNodes() < 5)
        return;

    vector<Edge> bridges[g.getMaxNode()];
    auto vor = VoronoiPartition(g, tr);
    bool isKey[tr.g->getMaxNode()];

    for(node_id n=0; n < tr.g->getMaxNode(); n++) {
        isKey[n] = (n < numTerminals || tr.g->nb[n].size() > 2);
    }
    isKey[tr.root] = true;


    // Find bridging edges
    auto edgeIt = g.findEdges();
    while(edgeIt.hasElement()) {
        auto edge = *edgeIt;
        auto t1 = vor.getClosest(edge.u);
        auto t2 = vor.getClosest(edge.v);
        // Is indeed a bridge
        if (t1.t != t2.t) {
            bridges[t1.t].emplace_back(edge.u, edge.v, edge.cost);
            bridges[t2.t].emplace_back(edge.u, edge.v, edge.cost);
        }
        ++edgeIt;
    }

    // Find key paths. This is a long process.
    vector<node_id> q;
    node_id parents[g.getMaxNode()];
    parents[tr.root] = g.getMaxNode();
    vector<node_id> kvq;
    unordered_map<node_id, unordered_set<node_id>> subsets;
    unordered_set<node_id> pinned;
    unordered_set<node_id> closed;

    // Extract the key vertex tree. Establish child-parent between all vertices.
    q.push_back(tr.root);
    while(!q.empty()) {
        auto v = q.back();
        q.pop_back();
        for(auto& nb: tr.g->nb[v]) {
            if (nb.first != parents[v]) {
                q.push_back(nb.first);
                parents[nb.first] = v;
                // Do not process the root, as there is no keypath from the root up
                if ((isKey[nb.first] || tr.g->nb[nb.first].size() == 1) && nb.first != tr.root) {
                    kvq.push_back(nb.first);
                }
            }
        }
    }

    // Iterate over the whole structure in DFS order
    while(! kvq.empty()) {
        // Get current node
        node_id n = kvq.back();
        kvq.pop_back();
        // find path
        unordered_set<node_id> intermediaries;
        vector<node_id> path;
        path.push_back(n);
        node_id p = parents[n];
        cost_id pathCost = 0;
        bool foundPinned = false;
        for(; !isKey[p]; p = parents[p]) {
            pathCost += g.nb[path.back()][p];
            path.push_back(p);
            intermediaries.insert(p);
            if (pinned.count(p) > 0)
                foundPinned = true;
        }
        pathCost += g.nb[path.back()][p];
        path.push_back(p);
        vector<Edge>* p1 = nullptr;
        vector<Edge>* p2 = nullptr;
        node_id p1end = 0;
        node_id p2end = 0;

        // Extend subset of parent key node
        auto np = path.back();
        subsets[n].insert(n);
        subsets[np].reserve(subsets[n].size() + path.size() + subsets[n].size());
        subsets[np].insert(subsets[n].begin(), subsets[n].end());
        subsets[np].insert(path.begin(), path.end());

        // Do not move pinned elements and recheck if still important
        if (foundPinned || (n >= numTerminals && tr.g->nb[n].size() <= 2)) {
            continue;
        }

        // Repair diagram -> move nodes previously assigned to intermediaries to other nodes (simulate removal)
        vor.repair(intermediaries);

        // Treat n as in intermediary from here on! (Do not move before repair!)
        intermediaries.insert(n);
        Bridge minBridge(MAXCOST, 0, 0, 0);

        // Repeat vor intermediaries. Slightly different from above
        for(auto im: intermediaries) {
            for(size_t i=0; i < bridges[n].size(); i++) {
                auto& cBridge = bridges[n][i];
                auto r1 = vor.getClosest(cBridge.u);
                auto r2 = vor.getClosest(cBridge.v);
                // We know that one of the two nodes is in the current region, make sure not both (would disconnect graph)
                bool uIn = subsets[n].count(r1.t) > 0;
                bool vIn = subsets[n].count(r2.t) > 0;

                // Since the region is getting larger, remove invalid bridges for key vertices
                if (im == n && uIn && vIn) {
                    if (i != bridges[n].size() - 1)
                        std::swap(bridges[n][i], bridges[n].back());
                    bridges[n].pop_back();
                    i--;
                }
                else if (uIn ^ vIn) {
                    cost_id totalCost = cBridge.cost + r1.costEntry.cost + r2.costEntry.cost;
                    if (totalCost < minBridge.total && closed.count(r1.t) == 0 && closed.count(r2.t) == 0) {
                        delete p1;
                        delete p2;
                        p1 = vor.getPath(cBridge.u);
                        p2 = vor.getPath(cBridge.v);
                        p1end = r1.t;
                        p2end = r2.t;

                        minBridge = Bridge(totalCost, cBridge.u, cBridge.v, cBridge.cost);
                    }
                }
            }
        }

        // Found cheaper key path
        if (minBridge.total < pathCost || (favorNew && minBridge.total == pathCost)) {
            // Remove old path
            for(size_t idx=0; idx < path.size() - 1; idx++) {
                tr.g->removeEdge(path[idx], path[idx+1]);
            }
            assert(tr.g->getNodes().count(p1end) > 0 || p1end < numTerminals || p1end == tr.root);
            assert(tr.g->getNodes().count(p2end) > 0 || p2end < numTerminals || p2end == tr.root);
            int total = 0;
            // new path
            for(auto& e: *p1) {
                tr.g->addEdge(e.u, e.v, e.cost);
                total += e.cost;
            }
            for(auto& e: *p2) {
                tr.g->addEdge(e.u, e.v, e.cost);
                total += e.cost;
            }
            tr.g->addEdge(minBridge.e.u, minBridge.e.v, minBridge.e.cost);
            total += minBridge.e.cost;
            assert(minBridge.total == total);

            // Forbid subset and intermediaries from  change
            for(auto f: subsets[n])
                closed.insert(f);
            for(auto im: intermediaries)
                closed.insert(im);

            // Pin endpoints
            pinned.insert(p1end);
            pinned.insert(p2end);
            assert(tr.g->checkConnectedness(0, false));
        }

        // Move to parent key vertex
        //TODO: Reserve for intermediaries as well? Maybe count them above?
        bridges[np].reserve(bridges[np].size() + bridges[n].size());
        for(auto im: intermediaries) {
            std::move(begin(bridges[im]), end(bridges[im]), back_inserter(bridges[np]));
        }

        vor.reset();

        delete p1;
        delete p2;
    }
    tr.cost = tr.g->getCost();
}

void steiner::LocalOptimization::keyVertexDeletion(Graph& g, SteinerResult& tr, node_id nTerminals) {
    // Find predecessors for common ancestor finding by performing a DFS traversal
    NodeWithCost p[g.getMaxNode()];
    p[tr.root] = NodeWithCost(tr.root, 0);
    unordered_map<node_id, unordered_set<node_id>> subsets;
    unordered_set<node_id> keyChildren[g.getMaxNode()];
    vector<node_id> intermediaries[g.getMaxNode()];
    unordered_set<node_id> pinned;
    unordered_set<node_id> closed;

    bool isKey[tr.g->getMaxNode()];
    for(node_id n=0; n < tr.g->getMaxNode(); n++) {
        isKey[n] = (n < nTerminals || tr.g->nb[n].size() > 2);
    }
    isKey[tr.root] = true;

    vector<node_id> q;
    q.push_back(tr.root);
    vector<node_id> kvq;
    kvq.push_back(tr.root);

    while(! q.empty()) {
        auto n = q.back();
        auto& pr = p[n];
        q.pop_back();

        for(auto nb: tr.g->nb[n]) {
            if (nb.first != pr.node) {
                p[nb.first] = NodeWithCost(n, pr.cost + 1);
                q.push_back(nb.first);
                if (isKey[nb.first] || tr.g->nb[nb.first].size() == 1)
                    kvq.push_back(nb.first);
            }
        }
    }

    vector<Edge> bridges[tr.g->getMaxNode()];
    vector<Edge> horizontal[tr.g->getMaxNode()];

    auto vor = VoronoiPartition(g, tr);


    auto it = g.findEdges();
    while(it.hasElement()) {
        auto e = *it;
        auto t1 =  vor.getClosest(e.u);
        auto t2 = vor.getClosest(e.v);

        if (t1.t != t2.t) {
            bridges[t1.t].emplace_back(e.u, e.v, e.cost);
            bridges[t2.t].emplace_back(e.u, e.v, e.cost);

            // Find common ancestor
            auto pred1 = NodeWithCost(t1.t, p[t1.t].cost + 1);
            auto pred2 = NodeWithCost(t2.t, p[t2.t].cost + 1);
            while(pred1.node != pred2.node) {
                if (pred1.cost > pred2.cost)
                    pred1 = p[pred1.node];
                else
                    pred2 = p[pred2.node];
            }

            // If common ancestor is higher up -> horizontal edge
            if (pred1.node != t1.t && pred1.node != t2.t)
                horizontal->push_back(e);
        }
        ++it;
    }

    while(! kvq.empty()) {
        auto n = kvq.back();
        kvq.pop_back();
        vector<node_id> parentPath;
        parentPath.push_back(n);

        // Find path to parent key node
        if (n != tr.root) {
            auto pred = p[n].node;
            while (! isKey[pred]) {
                parentPath.push_back(pred);
                intermediaries[n].push_back(pred);
                pred = p[pred].node;
            }
            parentPath.push_back(pred);
            keyChildren[pred].insert(n);
        }

        // Compute subnodes and intermediaries
        for(auto kc : keyChildren[n]) {
            subsets[n].insert(subsets[kc].begin(), subsets[kc].end());
        }
        subsets[n].insert(n);

        // Can't remove terminals, need at least more than one key child
        bool skip = false;

        if (n < nTerminals || keyChildren[n].size() <= 1)
            skip = true;

        // compute intermediaries
        unordered_set<node_id> allIntermediaries;
        unordered_set<node_id> childIntermediaries;

        if (! skip) {
            allIntermediaries.insert(intermediaries[n].begin(), intermediaries[n].end());
            for (auto kc : keyChildren[n]) {
                allIntermediaries.insert(intermediaries[kc].begin(), intermediaries[kc].end());
                childIntermediaries.insert(intermediaries[kc].begin(), intermediaries[kc].end());
            }
            allIntermediaries.insert(n);

            // Check if any pinned vertices are in the intermediaries
            bool foundPinned = false;
            for (auto cP : pinned) {
                if (allIntermediaries.count(cP) > 0) {
                    foundPinned = true;
                    break;
                }
            }
            if (foundPinned)
                skip = true;
        }

        if (!skip) {
            vor.repair(allIntermediaries);

            // Find candidate edges
            vector<Edge> candidateEdges;
            for (auto &ce: horizontal[n]) {
                auto r1 = vor.getClosestNoTmp(ce.u);
                auto r2 = vor.getClosestNoTmp(ce.v);
                if (childIntermediaries.count(r1.t) == 0 && childIntermediaries.count(r2.t) == 0)
                    candidateEdges.push_back(ce);
            }

            // Find min bridge and clean up bridges
            for (auto ck: keyChildren[n]) {
                Edge minBridge;
                cost_id minBridgeCost = MAXCOST;
                for (auto i = 0; i < bridges[ck].size(); i++) {
                    auto &ce = bridges[ck][i];
                    auto &t1 = vor.getClosestNoTmp(ce.u);
                    auto &t2 = vor.getClosestNoTmp(ce.v);
                    bool uInSub = t1.t != n && subsets[n].count(t1.t) > 0;
                    bool uInInt = allIntermediaries.count(t1.t) > 0;
                    bool vInSub = t2.t != n && subsets[n].count(t2.t) > 0;
                    bool vInInt = allIntermediaries.count(t2.t) > 0;


                    // We want bridges where one endpoint is above n and one is below n, so that connect the parent component
                    // with one of the child components
                    if (!((uInSub && !vInSub && !vInInt) || (vInSub && !uInSub && !uInInt))) {
                        if (i < bridges[ck].size())
                            swap(bridges[ck][i], bridges[ck].back());
                        bridges[ck].pop_back();
                        i--;
                    } else {
                        auto cost = t1.costEntry.cost + t2.costEntry.cost + ce.cost;
                        if (cost < minBridgeCost) {
                            minBridge = ce;
                        }
                    }
                }
                if (minBridgeCost < MAXCOST)
                    candidateEdges.push_back(minBridge);
            }

            // Inline lambda that finds out, in which neighbor component the region is
            auto findComponent = [&subsets, n, &keyChildren, &parentPath](auto st) {
                if (subsets[n].count(st) > 0) {
                    for (auto ck: keyChildren[n]) {
                        if (subsets[ck].count(st) > 0) {
                            return ck;
                        }
                    }
                }
                return parentPath.back();
            };

            for (auto im: allIntermediaries) {
                for (auto &ce: bridges[im]) {
                    // Determine in which subcomponent the endpoints are
                    auto &t1 = vor.getClosest(ce.u);
                    auto &t2 = vor.getClosest(ce.v);
                    node_id c1 = findComponent(t1.t);
                    node_id c2 = findComponent(t2.t);
                    if (c1 != c2)
                        candidateEdges.push_back(ce);
                }
            }

            // Found all possible edges, compute MST
            Graph subg = Graph();
            //Graph subg = Graph(g.getMaxNode());
            unordered_map<Edge, Edge> edgeMap;
            // First compute shortest paths and add resulting edge to graph
            for (auto &ce: candidateEdges) {
                auto &t1 = vor.getClosest(ce.u);
                auto &t2 = vor.getClosest(ce.v);

                node_id c1 = findComponent(t1.t);
                node_id c2 = findComponent(t2.t);

                cost_id total = t1.costEntry.cost + t2.costEntry.cost + ce.cost;
                if (closed.count(t1.t) == 0 && closed.count(t2.t) == 0) {
                    if (subg.addMappedEdge(c1, c2, total))
                        edgeMap.emplace(Edge(subg.getNodeMapping(c1), subg.getNodeMapping(c2), total), ce);
                }
            }

            // If we don't have enough bridges, the graph may be lacking, check first
            if (keyChildren[n].size() + 1 == subg.getNumNodes() && subg.checkConnectedness(0, false)) {
                auto *mst = subg.mst();
                cost_id mst_cost = mst->getCost();

                // Function that finds a path between two nodes
                auto findEdges = [&p, &g](node_id start, node_id end, vector<Edge> &e) {
                    node_id prev = start;

                    while (prev != end) {
                        node_id cn = p[prev].node;
                        e.emplace_back(prev, cn, g.nb[prev][cn]);
                        prev = cn;
                    }
                };

                vector<Edge> childEdges;
                findEdges(n, parentPath.back(), childEdges);
                for (auto kv: keyChildren[n])
                    findEdges(kv, n, childEdges);
                cost_id originalCost = 0;
                for (auto &ce: childEdges)
                    originalCost += ce.cost;

                // Found a better replacement?
                if (originalCost > mst_cost) {
                    for (auto &ce: childEdges)
                        tr.g->removeEdge(ce.u, ce.v);
                    // Use mapped edges to avoid huge overhead
                    auto mstEdges = mst->findEdges();
                    while (mstEdges.hasElement()) {
                        auto ce = *mstEdges;
                        assert(edgeMap.count(ce) > 0);
                        auto &originalE = edgeMap[ce];
                        tr.g->addEdge(originalE.u, originalE.v, originalE.cost);
                        auto *p1 = vor.getPath(originalE.u);
                        auto *p2 = vor.getPath(originalE.v);
                        auto t1 = vor.getClosest(originalE.u).t;
                        auto t2 = vor.getClosest(originalE.v).t;

                        for (auto &se: *p1)
                            tr.g->addEdge(se.u, se.v, se.cost);
                        for (auto &se: *p2)
                            tr.g->addEdge(se.u, se.v, se.cost);

                        delete p1;
                        delete p2;
                        pinned.insert(t1);
                        pinned.insert(t2);
                        ++mstEdges;
                    }

                    closed.reserve(closed.size() + subsets[n].size() + allIntermediaries.size());
                    closed.insert(subsets[n].begin(), subsets[n].end());
                    closed.insert(allIntermediaries.begin(), allIntermediaries.end());

                    assert(tr.g->checkConnectedness(0, false));
                }


                delete mst;
            }
        }

        vor.reset();

        // TODO: Use reserve?
        for(auto kv: keyChildren[n]) {
            subsets[n].insert(intermediaries[kv].begin(), intermediaries[kv].end());
            std::move(begin(bridges[kv]), end(bridges[kv]), back_inserter(bridges[n]));
            for(auto im: intermediaries[kv]) {
                std::move(begin(bridges[im]), end(bridges[im]), back_inserter(bridges[n]));
            }
        }
    }
    tr.cost = tr.g->getCost();
}

steiner::VoronoiPartition::VoronoiPartition(steiner::Graph &g, steiner::SteinerResult &tr) : g_(g) {
    regions_ = new unordered_map<node_id, NodeWithCost>[tr.g->getMaxNode()];
    closest_ = new ClosestEntry*[g.getMaxNode()];
    regionsTmp_ = new unordered_map<node_id, NodeWithCost>[tr.g->getMaxNode()];
    closestTmp_ = new ClosestEntry*[g.getMaxNode()];

    auto q = priority_queue<VoronoiQueueEntry>();
    bool visited[g.getMaxNode()];

    for(auto i=0; i < g.getMaxNode(); i++){
        visited[i] = false;
        closest_[i] = nullptr;
        closestTmp_[i] = nullptr;
    }

    for(auto n: tr.g->getNodes()) {
        q.emplace(0, n, n, n);
    }

    while(!q.empty()) {
        auto elem = q.top();
        q.pop();

        if (! visited[elem.n]) {
            visited[elem.n] = true;
            auto newElement = regions_[elem.start].emplace(std::piecewise_construct, forward_as_tuple(elem.n), forward_as_tuple(elem.predecessor, elem.c));
            closest_[elem.n] = new ClosestEntry(elem.start, newElement.first->second);

            for(auto& nb: g.nb[elem.n]) {
                if (!visited[nb.first]) {
                    q.emplace(elem.c + nb.second, nb.first, elem.start, elem.n);
                }
            }
        }
    }

}

void steiner::VoronoiPartition::reset() {
    for(auto i=0; i < g_.getMaxNode(); i++){
        regionsTmp_[i].clear();
        delete closestTmp_[i];
        closestTmp_[i] = nullptr;
    }
}

void steiner::VoronoiPartition::repair(unordered_set<node_id>& intermediaries) {
    reset();

    if (intermediaries.empty())
        return;

    unordered_set<node_id> repairNodes;
    bool visited[g_.getMaxNode()];
    for(node_id i=0; i < g_.getMaxNode(); i++) {
        visited[i] = false;
    }

    // Reassign all nodes associated with the "removed" voronoi centers
    for(auto n: intermediaries) {
        for(auto& n2: regions_[n]) {
            repairNodes.insert(n2.first);
        }
    }

    priority_queue<VoronoiQueueEntry> q;

    // Initialize dijkstra. Boundary nodes are added with distance to center
    for(auto n: repairNodes) {
        // Find a neighbor that is not in the same region and try if it's associated center fits
        for(auto& n2: g_.nb[n]) {
            auto& cl = closest_[n2.first];
            if (intermediaries.count(cl->t) == 0) {
                q.emplace(n2.second + cl->costEntry.cost, n, cl->t, n2.first);
            }
        }
    }

    // Dijkstra, limited to dangling (repair) nodes
    while(!q.empty()) {
        auto elem = q.top();
        q.pop();
        if (! visited[elem.n]) {
            visited[elem.n] = true;
            auto newElement = regionsTmp_[elem.start].emplace(std::piecewise_construct, forward_as_tuple(elem.n), forward_as_tuple(elem.predecessor, elem.c));
            closestTmp_[elem.n] = new ClosestEntry(elem.start, newElement.first->second);

            for(auto& nb: g_.nb[elem.n]) {
                if (!visited[nb.first] && repairNodes.count(nb.first) > 0) {
                    q.emplace(elem.c + nb.second, nb.first, elem.start, elem.n);
                }
            }
        }
    }
}

vector<steiner::Edge>* steiner::VoronoiPartition::getPath(node_id n) {
    auto& t = getClosest(n).t;
    auto* r = (!regionsTmp_[t].empty()) ? &regionsTmp_[t] : &regions_[t];
    bool switched = regionsTmp_[t].empty();

    auto* path = new vector<Edge>;

    while(t != n) {
        // Switch from new assignment to old, if no new assignments are available...
        if (! switched && r->count(n) == 0) {
            r = &regions_[t];
            switched = true;
        }

        assert(r->count(n) > 0);
        auto& prev = (*r)[n];
        path->emplace_back(n, prev.node, g_.nb[n][prev.node]);
        n = prev.node;
    }

    return path;
}
