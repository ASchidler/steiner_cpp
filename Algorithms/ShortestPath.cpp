//
// Created on 1/24/20.
//

#include "ShortestPath.h"
#include <random>
#include "LocalOptimization.h"
#include "../Reductions/Reducer.h"

bool steiner::ShortestPath::hasRun = false;
node_id steiner::ShortestPath::bestRoot = 0;
cost_id steiner::ShortestPath::bestResult = MAXCOST;

std::shared_ptr<steiner::SteinerResult> steiner::ShortestPath::calculate(node_id root, Graph& g, node_id nTerminals) {
    if (g.getNumNodes() == 1) {
        auto* miniTr = new Graph(g.getMaxNode());
        miniTr->addUnmappedNode(root);
        return make_shared<SteinerResult>(0, miniTr, root);
    }
    Graph tr(g.getMaxNode());
    tr.addUnmappedNode(root);

    node_id nRemaining = nTerminals;
    bool remaining[nTerminals];
    cost_id costs[g.getMaxNode()];
    bool added[g.getMaxNode()];
    node_id prev[g.getMaxNode()];

    Queue<DoubleCostEntry> q(g.getMaxKnownDistance());

    for(int i=0; i < nTerminals; i++)
        remaining[i] = true;

    for(int i=0; i < g.getMaxNode(); i++) {
        costs[i] = MAXCOST;
        added[i] = false;
    }

    if (root < nTerminals) {
        nRemaining--;
        remaining[root] = false;
    }
    added[root] = true;
    q.emplace(0, root, 0, 0);

    while (nRemaining > 0 && !q.empty()) {
        auto elem = q.dequeue();

        if (elem.node < nTerminals && remaining[elem.node]) {
            remaining[elem.node] = false;
            nRemaining--;

            // Backtrack path
            node_id cNode = elem.node;
            node_id cPred = prev[elem.node];
            vector<node_id> cache;
            while (!added[cNode]) {
                tr.addEdge(cNode, cPred, g.nb[cNode][cPred]);
                added[cNode] = true;
                cache.push_back(cNode);

                cNode = cPred;
                cPred = prev[cNode];
            }
            // Reexpand with 0 base cost (separate step, otherwise prev will be overwritten while path finding)
            for(const auto c: cache) {
                for(auto& v: g.nb[c]) {
                    if (v.second < costs[v.first] && !added[v.first]) {
                        q.emplace(v.second, v.first, v.second, v.second);
                        costs[v.first] = v.second;
                        prev[v.first] = c;
                    }
                }
            }

        } else {
            // Normal Dijkstra expand
            for(const auto& v: g.nb[elem.node]) {
                auto total = elem.totalCost + v.second;
                if (total < costs[v.first] && !added[v.first]) {
                    q.emplace(total, v.first, total, v.second);
                    costs[v.first] = total;
                    prev[v.first] = elem.node;
                }
            }
        }
    }

    if (nRemaining > 0)
        return nullptr;

    auto* mst = tr.mst();
    bool changed = true;
    while (changed) {
        changed = false;
        auto n = mst->getNodes().begin();
        while(n != mst->getNodes().end()) {
            if (mst->nb[*n].size() == 1 && *n != root && *n >= nTerminals) {
                n = mst->removeNode(n);
                changed = true;
            }
            else
                ++n;
        }
    }

    auto result = make_shared<SteinerResult>(mst->getCost(), mst, root);

    ShortestPath::hasRun = true;
    if (ShortestPath::bestResult > result->cost) {
        ShortestPath::bestResult = result->cost;
        if (root < nTerminals)
            ShortestPath::bestRoot = root;
    }
    return result;
}

void steiner::ShortestPath::resetPool(node_id nTerminals) {
    int ct = 0;
    int cnt = 0;

    for(const auto& sol: resultPool_) {
        if (sol->root < nTerminals && ct < 5) {
            terminalRoots[ct] = sol->root;
        } else if (sol->root >= nTerminals && cnt < 3) {
            nonTerminalRoots[cnt] = sol->root;
        }
    }
    // Avoid duplicates
    for(; ct < 5; ct++) {
        for (int i=0; i < ct; i++) {
            if (terminalRoots[i] == terminalRoots[ct]) {
                terminalRoots[ct] = MAXNODE;
                break;
            }
        }
    }
    for(; cnt < 3; cnt++) {
        for (int i=0; i < cnt; i++) {
            if (nonTerminalRoots[i] == nonTerminalRoots[cnt]) {
                nonTerminalRoots[cnt] = MAXNODE;
                break;
            }
        }
    }

    resultPool_.clear();
}

void steiner::ShortestPath::addToPool(const std::shared_ptr<SteinerResult>& result) {
    bool smaller = false;
    bool handled = false;
    lowestBound_ = min(lowestBound_, result->cost);

    // Check if there is a larger result
    for(size_t i=0; i < resultPool_.size(); i++) {
        auto r = resultPool_[i];
        // We want only one result per root
        if (r->root == result->root) {
            if (r->cost <= result->cost) {
                return;
            } else {
                resultPool_[i] = result;
                handled = true;
                break;
            }
        } else if (r->cost > result->cost) {
            smaller = true;
            break;
        }
    }

    if (!handled) {
        if (resultPool_.size() < poolSize_)
            resultPool_.push_back(result);
        else if (!smaller) {
            return;
        } else {
            resultPool_.back() = result;
        }
    }

    // Bubble up to keep sorted
    for(size_t i=resultPool_.size() - 1; i > 0; i--) {
        if (resultPool_[i]->cost < resultPool_[i - 1]->cost)
            swap(resultPool_[i], resultPool_[i-1]);
    }
}

void steiner::ShortestPath::findAndAdd(steiner::Graph &g, node_id nTerminals, node_id nSolutions) {
    auto roots = selectRoots(g, nTerminals, nSolutions);

    for(auto r: roots) {
        auto result = ShortestPath::calculate(r, g, nTerminals);
        if (result != nullptr)
            addToPool(result);
    }
}

unordered_set<node_id> steiner::ShortestPath::selectRoots(steiner::Graph &g, node_id nTerminals, node_id nSolutions) {
    // Select roots
    nSolutions = min(nSolutions, g.getNumNodes());
    unordered_set<node_id> roots;

    // Determine how many terminal and non-terminal roots we want
    node_id numTerminalRoots = min(static_cast<node_id>(0.7 * nSolutions), nTerminals);
    node_id numNonTerminalRoots = min(g.getNumNodes() - nTerminals, nSolutions - numTerminalRoots);
    // This may happen, if there are fewer non-terminals than we would need
    if (numNonTerminalRoots + numTerminalRoots < nSolutions)
        numTerminalRoots += nSolutions - numNonTerminalRoots - numTerminalRoots;

    // Select roots from terminals
    if (numTerminalRoots >= nTerminals) { // Easy case, select all terminals
        for(node_id i=0; i < nTerminals; i++)
            roots.insert(i);
    } else { // In this case mix best roots in and choose the rest at random
        node_id tAdded = 0;
        // Use best roots
        if (!resultPool_.empty()) {
            for(const auto& r: resultPool_) {
                if (r->root < nTerminals) {
                    roots.insert(r->root);
                    tAdded++;
                    if (tAdded >= 5 || tAdded >= numTerminalRoots)
                        break;
                }
            }
        } else {
            for(int i=0; i < 5 && tAdded < numTerminalRoots; i++) {
                if (terminalRoots[i] < nTerminals) {
                    roots.insert(terminalRoots[i]);
                    tAdded++;
                }
            }
        }
        for(; tAdded < numTerminalRoots; tAdded++) {
            auto newT = random() % nTerminals;
            while(roots.count(newT) > 0) {
                newT++;
                if (newT >= nTerminals)
                    newT = 0;
            }
            roots.insert(newT);
        }
    }

    // Select non-terminals
    if(numNonTerminalRoots >= g.getNumNodes() - nTerminals) {
        for(auto n: g.getNodes()) {
            if (n >= nTerminals)
                roots.insert(n);
        }
    } else {
        int ntAdded = 0;
        if (!resultPool_.empty()) {
            for(const auto& r: resultPool_) {
                if (r->root >= numTerminalRoots) {
                    if (g.getNodes().count(r->root) > 0) {
                        roots.insert(r->root);
                        ntAdded++;
                        if (ntAdded >= numNonTerminalRoots || ntAdded >= 3)
                            break;
                    }
                }
            }
        } else {
            for (int i=0; i < 3 && ntAdded < numNonTerminalRoots; i++) {
                if (g.getNodes().count(nonTerminalRoots[i]) > 0) {
                    if (g.getNodes().count(nonTerminalRoots[i]) > 0) {
                        roots.insert(nonTerminalRoots[i]);
                        ntAdded++;
                    }
                }
            }
        }

        for(; ntAdded < numNonTerminalRoots; ntAdded++) {
            auto newNt = numTerminalRoots + (random() % (g.getMaxNode() - nTerminals));
            while(roots.count(newNt) > 0 || g.getNodes().count(newNt) == 0) {
                newNt++;
                if (newNt >= g.getMaxNode())
                    newNt = nTerminals;
            }
            roots.insert(newNt);
        }
    }

    return roots;
}

void steiner::ShortestPath::recombine(node_id nSolutions, node_id nTerminals) {
    // In this case there is no point...
    if (nTerminals == 1)
        return;
    
    // Do poolsize and then always halve...
    // The idea is to have (n is poolsize): n n/2 n/2 n/4 n/4 n/4 n4 ...
    for(node_id i=1; i < nSolutions; i *= 2) {
        for(node_id j=i; j < 2 * i && j < nSolutions; j++) {
            // First select indices
            node_id numSolutions = max((node_id)3, (node_id) (resultPool_.size() / i));
            vector<node_id> solutionIndices;
            if (numSolutions == resultPool_.size()) {
                for (int idx = 0; idx < resultPool_.size(); idx++) {
                    solutionIndices.push_back(idx);
                }
            } else {
                solutionIndices.push_back(0);
                for (int idx = 0; idx < numSolutions-1; idx++) {
                    auto newIdx = random() % resultPool_.size();
                    // Do not avoid duplicates
                    solutionIndices.push_back(newIdx);
                }
            }

            // Next build graph from vertices and edges in those solutions
            node_id maxNode = 0;
            for (auto s: solutionIndices)
                maxNode = max(maxNode, resultPool_[s]->g->getMaxNode());
            cost_id counter[maxNode];
            cost_id maxCount=0;
            cost_id mult = 0;
            for(int cNode=0; cNode < maxNode; cNode++)
                counter[cNode] = 0;

            Graph g;
            g.addUnmappedNode(0);
            for (auto s: solutionIndices) {
                for (auto n: resultPool_[s]->g->getNodes()) {
                    for (auto &nb: resultPool_[s]->g->nb[n]) {
                        if (n < nb.first) {
                            g.addEdge(n, nb.first, nb.second);
                            maxCount = max(maxCount, ++counter[n]);
                            maxCount = max(maxCount, ++counter[nb.first]);
                        }
                    }
                }
            }

            // Reduce away suboptimal components
            SteinerInstance s(&g, nTerminals);
            auto red = Reducer::getMinimalReducer(&s, false);
            red.reduce();

            auto edgeIt = g.findEdges();
            while(edgeIt.hasElement()) {
                auto e = *edgeIt;
                g.nb[e.u][e.v] += mult * (2 * maxCount - counter[e.u] - counter[e.v]);
                g.nb[e.v][e.u] += mult * (2 * maxCount - counter[e.u] - counter[e.v]);
                ++edgeIt;
            }

            // Compute RSP
            // TODO: Randomize r?
            for (auto r=0; r < s.getNumTerminals() && r < 5; r++) {
                auto result = ShortestPath::calculate(r, g, s.getNumTerminals());
                ShortestPath::optimizeSolution(g, result, s.getNumTerminals());

                auto edgeItA = result->g->findEdges();
                while(edgeItA.hasElement()) {
                    auto e = *edgeItA;
                    result->g->nb[e.u][e.v] -= mult * (2 * maxCount - counter[e.u] - counter[e.v]);
                    result->g->nb[e.v][e.u] -= mult * (2 * maxCount - counter[e.u] - counter[e.v]);
                    result->cost -= mult * (2 * maxCount - counter[e.u] - counter[e.v]);
                    ++edgeItA;
                }

                result->g->remap(g);
                red.reset();
                red.unreduce(result.get());
                addToPool(result);
            }
        }
    }
}

void steiner::ShortestPath::optimize(Graph& g, node_id nSolutions, node_id nTerminals) {
    for(int i=0; i < nSolutions && i < resultPool_.size(); i++) {
        auto r = resultPool_[i];
        LocalOptimization::vertexInsertion(&g, *r, nTerminals);
        LocalOptimization::pathExchange(g, *r, nTerminals, true);
        LocalOptimization::keyVertexDeletion(g, *r, nTerminals);

        optimizeSolution(g, r, nTerminals);
        // Remove degree 1 non-terminals
        bool changed = true;
        while (changed) {
            changed = false;
            auto nit = r->g->getNodes().begin();
            while(nit != r->g->getNodes().end()) {
                if (r->g->nb[*nit].size() == 1 && *nit != r->root && *nit >= nTerminals) {
                    nit = r->g->removeNode(nit);
                    changed = true;
                }
                else
                    ++nit;
            }
        }
        lowestBound_ = min(lowestBound_, r->cost);

        // Maintain ordering
        if (i > 0 && r->cost < resultPool_[i - 1]->cost)
            swap(resultPool_[i], resultPool_[i-1]);
    }
}

void steiner::ShortestPath::optimizeSolution(Graph& g, const std::shared_ptr<SteinerResult>& r, node_id nTerminals) {
    LocalOptimization::vertexInsertion(&g, *r, nTerminals);
    LocalOptimization::pathExchange(g, *r, nTerminals, true);
    LocalOptimization::keyVertexDeletion(g, *r, nTerminals);
    // Remove degree 1 non-terminals
    bool changed = true;
    while (changed) {
        changed = false;
        auto nit = r->g->getNodes().begin();
        while(nit != r->g->getNodes().end()) {
            if (r->g->nb[*nit].size() == 1 && *nit != r->root && *nit >= nTerminals) {
                nit = r->g->removeNode(nit);
                changed = true;
            }
            else
                ++nit;
        }
    }
}
