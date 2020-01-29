//
// Created by aschidler on 1/29/20.
//

#ifndef STEINER_DEGREE3REDUCTION_H
#define STEINER_DEGREE3REDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"

namespace  steiner {
    class Degree3Reduction : public Reduction {
    public:
        Degree3Reduction(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Degree3";
        }
    private:
        cost_id SubDijkstra(node_id u, node_id v, unordered_set<node_id>* ignoreNodes, cost_id limit);
    };

    class Degree3Distances {
    public:
        Degree3Distances(SteinerInstance* instance, node_id us[], node_id uLen, unordered_set<node_id>* ignore) :
                instance_(instance), ignore_(ignore){
            for(int i=0; i < uLen; i++)
                q_.emplace(us[i], 0);
        }

        cost_id get(node_id target, cost_id limit);

    private:
        SteinerInstance* instance_;
        priority_queue<NodeWithCost> q_;
        unordered_set<node_id>* ignore_;
        unordered_map<node_id, cost_id> dist_;
        cost_id cMax_ = 0;
    };
}


#endif //STEINER_DEGREE3REDUCTION_H
