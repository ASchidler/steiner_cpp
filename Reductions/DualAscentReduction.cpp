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

    // TODO: Root selection
    // TODO: Allow for non-terminal roots?
    node_id numRoots = min((node_id)2, instance->getNumTerminals());
    node_id roots[numRoots];
    DualAscentResult* results[numRoots];
    for(node_id t=0; t < numRoots; t++)
        results[t] = DualAscent::calculate(instance->getGraph(), t, nullptr, instance->getNumTerminals(), instance->getGraph()->getMaxNode());

    for(node_id t=0; t < numRoots; t++)
        track += reduceGraph(results[t]);

    for(node_id t=0; t < numRoots; t++)
        delete results[t];

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setDistanceState(SteinerInstance::invalid);
    }

    return track;
}

bool pairSort(const pair<int,int> &a,
              const pair<int,int> &b)
{
    return (a.first > b.first || (a.first == b.first && a.second > b.second));
}

node_id steiner::DualAscentReduction::reduceGraph(steiner::DualAscentResult* r) {
    r->g->findDistances(r->root);
    cost_id* dist = r->g->getDistances()[r->root];
    vector<pair<node_id, node_id>> candidates;
    node_id track = 0;

    cost_id limit = instance->getApproximation()->getCost() - r->bound;
    auto vor = voronoi(r->g, instance->getNumTerminals());

    for(node_id t=0; t < instance->getNumTerminals(); t++) {
        for(auto n: vor[t]) {
            if (dist[n.node] + n.cost > limit) {
                instance->removeNode(n.node);
                track++;
            }
            else { // Identify deletable edges
                auto nb = r->g->nb[n.node];
                for(auto& n2: nb) {
                    auto edgeCost = dist[n2.first] + r->g->nb[n2.first][n.node] + n.cost;
                    if (edgeCost > limit)
                        candidates.emplace_back(min(n.node, n2.first), max(n.node, n2.first));
                }

                // TODO: NTDK
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

    delete [] vor;
    return track;
}
