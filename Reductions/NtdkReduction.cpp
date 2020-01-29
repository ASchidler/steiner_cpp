//
// Created by aschidler on 1/28/20.
//

#include "NtdkReduction.h"
// Generally this would also be possible with dg > 4, experimentally the results where not that good
node_id steiner::NtdkReduction::reduce(node_id currCount, node_id prevCount) {
    node_id track = 0;

    auto n = instance->getGraph()->getNodes()->begin();
    while(n != instance->getGraph()->getNodes()->end()) {
        auto nb = instance->getGraph()->nb[*n];

        if (*n >= instance->getNumTerminals() && nb.size() > 2 && nb.size() <= limit_) { // Main loop
            bool trueForAll = false;

            cost_id edgeSum = 0;
            vector<Edge> edges;
            node_id ids[nb.size()];

            for (auto& v: nb)
                edgeSum += v.second;

            // Compute distances between neighbors
            int nbIdx=0;
            for (auto& v: nb) {
                ids[nbIdx++] = v.first;
                for (auto& w: nb) {
                    if (v.first < w.first) {
                        cost_id c = 0;
                        if (restricted_) {
                            c = instance->getSteinerDistance(v.first, w.first);
                        } else {
                            c = SteinerLength::calculateSteinerLength(v.first, w.first, instance->getGraph(),
                                    edgeSum, depthLimit_, false, instance->getNumTerminals(), instance->getGraph()->getMaxNode());
                        }
                        edges.emplace_back(v.first, w.first, c);
                    }
                }
            }

            // Check if the MST/edge cost condition is satisfied
            // Optimized version for degree 3
            if (nb.size() == 3) {
                auto dist1 = edges[0].cost + edges[1].cost;
                auto dist2 = edges[0].cost + edges[2].cost;
                auto dist3 = edges[1].cost + edges[2].cost;
                trueForAll = (dist1 <= edgeSum || dist2 <= edgeSum || dist3 <= edgeSum);
            } else if (nb.size() == 4) {
                // MST for all edges
                auto g = Graph();
                for (auto& e: edges)
                    g.addMappedEdge(e.u, e.v, e.cost);
                cost_id mstSum = g.mst_sum();
                trueForAll = mstSum <= edgeSum;

                // MST for subsets of size 3. We are interested in subsets of cardinality 3 or higher
                // In case of degree 4, we can use a single index, telling us which node to exclude.
                for(node_id exclude = 0; trueForAll && exclude < 4; exclude++) {
                    // Use same method as above. Calculate the three MST options and take minimum
                    auto thisEdgeSum = edgeSum - nb[ids[exclude]];
                    auto dist1 = edges[dg4Lookup[exclude][0]].cost + edges[dg4Lookup[exclude][1]].cost;
                    auto dist2 = edges[dg4Lookup[exclude][0]].cost + edges[dg4Lookup[exclude][2]].cost;
                    auto dist3 = edges[dg4Lookup[exclude][1]].cost + edges[dg4Lookup[exclude][2]].cost;
                    trueForAll = (dist1 <= thisEdgeSum || dist2 <= thisEdgeSum || dist3 <= thisEdgeSum);
                }
            } else {
                cout << "NTDK for degree over 4 is not implemented yet" << endl;
            }
            // Remove node, introduce edges
            if (trueForAll) {
                node_id edgeIdx = 0;
                for (auto& v: nb) {
                    for (auto& w: nb) {
                        if (v.first < w.first) {
                            // Verify that the edge would not be removed by long edge reduction in the next step
                            if (v.second + w.second <= edges[edgeIdx].cost) {
                                if (instance->addEdge(v.first, w.first, w.second + v.second)) {
                                    merge(*n, v.first, w.first, v.second, w.second);
                                }
                            }
                            edgeIdx++;
                        }
                    }
                }

                n = instance->removeNode(n);
                track++;
            } else {
                ++n;
            }
        } // main loop
        else{
            ++n;
        }
    }

    if (track > 0)
        instance->setApproximationState(SteinerInstance::invalid);

    return track;
}
