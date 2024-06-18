//
// Created by andre on 31.01.20.
//

#ifndef STEINER_LOCALOPTIMIZATION_H
#define STEINER_LOCALOPTIMIZATION_H
#include "../Graph.h"

using std::unordered_set;

namespace steiner {
    struct ClosestEntry {
        ClosestEntry(node_id t, NodeWithCost& costEntry) : t(t), costEntry(costEntry) {

        }
        node_id t;
        NodeWithCost& costEntry;
    };
    struct Bridge {
        Bridge(cost_id total, node_id u, node_id v, cost_id c) : total(total), e(Edge(u, v, c))  {

        }
        Edge e;
        cost_id total;
        bool operator<(const Bridge& p2) const {
            return total > p2.total;
        }
    };

    class VoronoiPartition {
    public:
        VoronoiPartition(Graph& g, SteinerResult& tr);
        ClosestEntry& getClosest(node_id n) {
            if (closestTmp_[n] != nullptr) {
                return *closestTmp_[n];
            }

            return *closest_[n];
        }
        ClosestEntry& getClosestNoTmp(node_id n) {
            return *closest_[n];
        }
        void reset();

        NodeWithCost& getRegionEntry(node_id n) {
            if (closestTmp_[n] != nullptr) {
                return closestTmp_[n]->costEntry;
            }
            return closest_[n]->costEntry;
        }
        bool isInRegion(node_id n, node_id r) {
            return regions_[r].count(n) > 0;
        }
        bool isInTmpRegion(node_id n, node_id r) {
            return regionsTmp_[r].count(n) > 0;
        }
        vector<Edge>* getPath(node_id n);
        void repair(unordered_set<node_id>& intermediaries);

    private:
        ClosestEntry** closest_;
        ClosestEntry** closestTmp_;
        unordered_map<node_id, NodeWithCost>* regions_;
        unordered_map<node_id, NodeWithCost>* regionsTmp_;
        Graph& g_;
        struct VoronoiQueueEntry {
            VoronoiQueueEntry(cost_id c, node_id n, node_id start, node_id pred):
            c(c), n(n), start(start), predecessor(pred) {}

            cost_id c;
            node_id n;
            node_id start;
            node_id predecessor;
            bool operator<(const VoronoiQueueEntry& p2) const {
                return c > p2.c;
            }

            bool operator>(const VoronoiQueueEntry& p2) const {
                return c > p2.c;
            }
        };



    };

    class LocalOptimization {
    public:
        static void vertexInsertion(Graph* dg, SteinerResult& tr, node_id nTerminals);
        static void pathExchange(Graph& g, SteinerResult& tr, node_id nTerminals, bool favorNew);
        static void keyVertexDeletion(Graph& g, SteinerResult& tr, node_id nTerminals);
    };

    struct KeyPath {
        vector<node_id> path;
        vector<KeyPath*> childPaths;

    };


}


#endif //STEINER_LOCALOPTIMIZATION_H
