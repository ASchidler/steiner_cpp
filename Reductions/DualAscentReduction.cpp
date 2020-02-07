//
// Created by aschidler on 1/30/20.
//

#include "DualAscentReduction.h"

node_id steiner::DualAscentReduction::reduce(node_id currCount, node_id prevCount) {
    if (instance->getNumTerminals() < 3)
        return 0;

    node_id track = 0;

    // Always request a new approximation for this reduction
    instance->requestApproximationState(SteinerInstance::invalid);

    // TODO: Define control parameters according to instance size

    node_id numRoots = min((node_id)50, instance->getNumTerminals());
    node_id roots[numRoots];
    chooseRoots(roots, numRoots);
    SteinerResult* results[numRoots];
    for(node_id t=0; t < numRoots; t++)
        results[t] = DualAscent::calculate(instance->getGraph(), roots[t], nullptr, instance->getNumTerminals(), instance->getGraph()->getMaxNode());

    node_id tracks[numRoots];
    for(node_id t=0; t < numRoots; t++) {
        tracks[t] = reduceGraph(results[t]);
        track += tracks[t];
    }

    selectRoots(results, numRoots, tracks);
    for(node_id t=0; t < numRoots; t++)
        delete results[t];

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setDistanceState(SteinerInstance::invalid);
        instance->setSteinerDistanceState(SteinerInstance::invalid);
    }
    enabled = track > 0;
    return track;
}

bool pairSort(const pair<int,int> &a,
              const pair<int,int> &b)
{
    return (a.first > b.first || (a.first == b.first && a.second > b.second));
}

node_id steiner::DualAscentReduction::reduceGraph(steiner::SteinerResult* r) {
    r->g->findDistances(r->root);
    cost_id* dist = r->g->getDistances()[r->root];
    vector<pair<node_id, node_id>> candidates;
    node_id track = 0;

    cost_id limit = instance->getUpperBound();
    limit -= r->cost;
    auto vor = voronoi(r->g, instance->getNumTerminals());

    for(node_id t=0; t < instance->getNumTerminals(); t++) {
        for(auto n: vor->closest[t]) {
            // Check if we can remove node
            if (dist[n.node] + n.cost > limit) {
                instance->removeNode(n.node);
                track++;
            }
            // NTDK test, does the node have maximum degree 2 in any steiner tree?
            else if (n.node >= instance->getNumTerminals() && dist[n.node] + vor->second[n.node].cost > limit) {
                if (instance->getGraph()->nb[n.node].size() <= 6 && instance->getGraph()->getNodes().count(dist[n.node]) > 0) {
                    for (auto &b: instance->getGraph()->nb[n.node]) {
                        for (auto &b2: instance->getGraph()->nb[n.node]) {
                            if (b.first < b2.first) {
                                if (instance->addEdge(b.first, b2.first, b.second + b2.second)) {
                                    merge(n.node, b.first, b2.first, b.second, b2.second);
                                }
                            }
                        }
                    }
                    instance->removeNode(n.node);
                    track++;
                }
            }
            // Can we remove some if the edges?
            else { // Identify deletable edges
                auto nb = r->g->nb[n.node];
                for(auto& n2: nb) {
                    // n.cost is the cost to reach the terminal. We seek root -> edge(n2, n) -> t
                    auto edgeCost = dist[n2.first] + r->g->nb[n2.first][n.node] + n.cost;
                    if (edgeCost > limit)
                        candidates.emplace_back(min(n.node, n2.first), max(n.node, n2.first));
                }
            }
        }
    }

    // The edge candidates have the smaller vertex first, after sorting they are next to each other
    sort(candidates.begin(), candidates.end(), pairSort);
    auto edgeIt = candidates.begin();
    while(edgeIt != candidates.end()) {
        auto next = (edgeIt + 1);
        if (next == candidates.end())
            break;

        if (edgeIt->first == next->first && edgeIt->second == next->second) {
            instance->removeEdge(edgeIt->first, edgeIt->second);
            ++edgeIt; // Skip one element
            track++;
        }
        ++edgeIt;
    }

    delete vor;
    return track;
}

void steiner::DualAscentReduction::chooseRoots(node_id *roots, node_id numRoots) {
    assert(numRoots <= instance->getNumTerminals());
    node_id rootsSelected = 0;
    node_id selectBest = min(2, numRoots/2);
    for(auto i=0; i < selectBest && rootsSelected < numRoots; i++) {
        if (bestRoots[i] < instance->getNumTerminals()) {
            roots[rootsSelected] = bestRoots[i];
            rootsSelected++;
        }
    }

    // Now select terminals
    for(; rootsSelected < numRoots; rootsSelected++) {
        node_id t = random() % instance->getNumTerminals();
        // Select next t that has not been selected yet
        for(auto j=0; j < rootsSelected; j++) {
            if (roots[j] == t) {
                t++;
                if (t >= instance->getNumTerminals())
                    t = 0;
                j=0;
            }
        }

        roots[rootsSelected] = t;
    }
}

void steiner::DualAscentReduction::selectRoots(steiner::SteinerResult** results, node_id numSolutions, const node_id *track) {
    // Okay so the idea is that we choose the two best roots in terms of bounds and the two best roots in terms of elimination
    cost_id bestBound = 0;
    node_id bestTrack = 0;

    for(node_id i=0; i < numSolutions; i++) {
        if(results[i]->cost > bestBound) {
            bestRoots[0] = results[i]->root;
            bestBound = results[i]->cost;
        } else if (track[i] > bestTrack) {
            bestRoots[1] = results[i]->root;
            bestTrack = track[i];
        }
    }
}
