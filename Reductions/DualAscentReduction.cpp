//
// Created by aschidler on 1/30/20.
//

#include "DualAscentReduction.h"
#include "../Reductions/Reducer.h"
#include "../Algorithms/GraphPruner.h"

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
    VoronoiDiagram* vors[numRoots];

    for(node_id t=0; t < numRoots; t++) {
        results[t] = DualAscent::calculate(instance->getGraph(), roots[t], nullptr, instance->getNumTerminals(),
                                           instance->getGraph()->getMaxNode());
        results[t]->g->findDistances(results[t]->root, instance->getGraph()->getMaxKnownDistance());
        vors[t] = VoronoiDiagram::create(results[t]->g, instance->getNumTerminals());
    }
    std::sort(results, results + numRoots, SteinerResult::cmp);

//    cout << "Before Prune Ascent " << instance->getApproximation().getLowest() << endl;
    //pruneAscent(results, numRoots, 20);
//    cout << "After Prune Ascent " << instance->getApproximation().getLowest() << endl;
    //cout << "Before Prune: " << instance->getApproximation().getLowest() << endl;
//    for(node_id t=0; t < numRoots && t < 5; t++){
//        prune(results[t]);
//    }
    //cout << "After Prune: " << instance->getApproximation().getLowest() << endl;

    node_id tracks[numRoots];
    for(node_id t=0; t < numRoots; t++) {
        tracks[t] = reduceGraph(results[t], *vors[t], instance, instance->getUpperBound());
        track += tracks[t];
    }

    selectRoots(results, numRoots, tracks);
    for(node_id t=0; t < numRoots; t++) {
        delete results[t];
        delete vors[t];
    }

    if (track > 0) {
        instance->setSteinerDistanceState(SteinerInstance::invalid);
        instance->setDistanceState(SteinerInstance::invalid);
        instance->setSteinerDistanceState(SteinerInstance::invalid);
    }
    enabled = track > 0;
    return track;
}

// TODO: Maybe get rid of dynamic bitset? Change it to integers and only if > 128 use some other, non-expensive form?
//TODO: Bucket queues... Queue wrapper that either uses buckets or heap depending on upper bound
node_id steiner::DualAscentReduction::reduceGraph(SteinerResult* r, VoronoiDiagram& vor, SteinerInstance* inst, cost_id bound) {
    if (!r->g->hasDistances())
        r->g->findDistances(r->root, instance->getGraph()->getMaxKnownDistance());

    cost_id* dist = r->g->getDistances()[r->root];
    node_id track = 0;

    cost_id limit = bound - r->cost;

    // Use r->g since the instance's nodes may change during iteration
    for(const auto n : r->g->getNodes()) {
        // Ensure that the node is still there, otherwise NTDK may add ghost edges...
        if (inst->getGraph()->nb[n].empty())
            continue;
        if (dist[n] + vor.closest[n].cost > limit) {
            inst->removeNode(n);
            track++;
        }
        // NTDK test, does the node is a non-terminal and have maximum degree 2 in any steiner tree?
        else if (n >= inst->getNumTerminals() && dist[n] + vor.second[n].cost > limit and
                inst->getGraph()->nb[n].size() <= 6 && inst->getGraph()->getNodes().count(dist[n]) > 0) {
            // Insert replacement edges
            for (const auto &b: inst->getGraph()->nb[n]) {
                for (const auto &b2: inst->getGraph()->nb[n]) {
                    if (b.first < b2.first) {
                        if (inst->addEdge(b.first, b2.first, b.second + b2.second)) {
                            merge(n, b.first, b2.first, b.second, b2.second);
                        }
                    }
                }
            }

            inst->removeNode(n);
            track++;
        }
        // Can we remove some if the edges?
        else {
            auto nb = r->g->nb[n];
            for(const auto& n2: nb) {
                // Try to avoid doing the check twice
                if (n < n2.first) {
                    // Must be over the lower bound from both sides
                    auto edgeCost1 = dist[n2.first] + r->g->nb[n2.first][n] + vor.closest[n].cost;
                    auto edgeCost2 = dist[n] + n2.second + vor.closest[n2.first].cost;

                    if (edgeCost1 > limit && edgeCost2 > limit) {
                        inst->removeEdge(n, n2.first);
                        track++;
                    }
                }
            }
        }
    }

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

void steiner::DualAscentReduction::prune(steiner::SteinerResult *r) {
    // Create a graph consisting of the edges in the dual ascent graph, we know that this is connected
    Graph g = Graph();
    auto edgeIt = r->g->findEdges();
    while(edgeIt.hasElement()) {
        auto e = *edgeIt;
        // findEdges returns undirected edges, so look in both directions
        if (r->g->nb[e.u][e.v] == 0 || r->g->nb[e.v][e.u] == 0)
            g.addEdge(e.u, e.v, instance->getGraph()->nb[e.u][e.v]);
        ++edgeIt;
    }

    // Use a limited reducer for this new sub-graph
    SteinerInstance s(&g, instance->getNumTerminals());
    GraphPruner p(s);

    // Now periodically lower the upper bound and apply bound based reduction
    unsigned int cnt = 0;
    do {
        p.reduce();

        //TODO: Make number of roots configurable?
        for(auto t=0; t < s.getNumTerminals() && t < 5; t++) {
            instance->getApproximation().addToPool(p.approximate(true));
        }

        // TODO: Make number of pruning cycles configurable
        // TODO: Make number of prunings configurable
        cnt++;
    // Do this several times or until the graph is disconnected
    } while(cnt < 4 && s.getNumTerminals() > 3 && p.prune());
}

void DualAscentReduction::pruneAscent(SteinerResult **results, node_id numSolutions, node_id numRuns) {
    bool stop = false;
    for(int cRun=1; cRun <= numRuns && !stop; cRun++) {
        cost_id counter[instance->getGraph()->getMaxNode()];
        cost_id maxCount=0;
        for(int cNode=0; cNode < instance->getGraph()->getMaxNode(); cNode++)
            counter[cNode] = 0;


        // TODO: Make upper limit configurable?
        int numSelect = min(numSolutions / numRuns, 15);
        stop = numSelect <= 1;

        if (numSelect > 0) {
            Graph g;
            for(int cResult=0; cResult < numSelect; cResult++) {
                auto edgeIt = results[random() % numSolutions]->g->findEdges();
                while(edgeIt.hasElement()) {
                    auto e = *edgeIt;
                    // findEdges returns undirected edges, so look in both directions
                    if (results[cResult]->g->nb[e.u][e.v] == 0 || results[cResult]->g->nb[e.v][e.u] == 0) {
                        g.addEdge(e.u, e.v, instance->getGraph()->nb[e.u][e.v]);
                        maxCount = max(maxCount, ++counter[e.u]);
                        maxCount = max(maxCount, ++counter[e.v]);
                    }
                    ++edgeIt;
                }
            }

            // Use a limited reducer for this new sub-graph
            SteinerInstance s(&g, instance->getNumTerminals());
            GraphPruner p(s);

            // Now periodically lower the upper bound and apply bound based reduction
            unsigned int cnt = 0;
            do {
                p.reduce();
                auto edgeIt = g.findEdges();
                while(edgeIt.hasElement()) {
                    auto e = *edgeIt;
                    g.nb[e.u][e.v] += 2 * maxCount - counter[e.u] - counter[e.v];
                    g.nb[e.v][e.u] += 2 * maxCount - counter[e.u] - counter[e.v];
                    ++edgeIt;
                }

                //TODO: Make number of roots configurable?
                for(auto t=0; t < s.getNumTerminals() && t < 5; t++) {
                    auto cResult = p.approximate(false);
                    auto edgeItA = cResult->g->findEdges();
                    while(edgeItA.hasElement()) {
                        auto e = *edgeItA;
                        cResult->g->nb[e.u][e.v] -= 2 * maxCount - counter[e.u] - counter[e.v];
                        cResult->g->nb[e.v][e.u] -= 2 * maxCount - counter[e.u] - counter[e.v];
                        cResult->cost -= 2 * maxCount - counter[e.u] - counter[e.v];
                        ++edgeItA;
                    }
                    p.unreduce(cResult);
                    instance->getApproximation().addToPool(cResult);
                }

                auto edgeIt2 = g.findEdges();
                while(edgeIt2.hasElement()) {
                    auto e = *edgeIt2;
                    g.nb[e.u][e.v] -= 2 * maxCount - counter[e.u] - counter[e.v];
                    g.nb[e.v][e.u] -= 2 * maxCount - counter[e.u] - counter[e.v];
                    ++edgeIt2;
                }

                // TODO: Make number of pruning cycles configurable
                // TODO: Make number of prunings configurable
                cnt++;
                // Do this several times or until the graph is disconnected
            } while(cnt < 4 && s.getNumTerminals() > 3 && p.prune());
        }
    }
}
