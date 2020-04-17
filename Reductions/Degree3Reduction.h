//
// Created by aschidler on 1/29/20.
//

#ifndef STEINER_DEGREE3REDUCTION_H
#define STEINER_DEGREE3REDUCTION_H
#include "Reduction.h"
#include "../Algorithms/SteinerLength.h"
#include "../Structures/Queue.h"

namespace  steiner {
    class Degree3Reduction : public Reduction {
    public:
        explicit Degree3Reduction(SteinerInstance *s) : Reduction(s) {
        }

        node_id reduce(node_id currCount, node_id prevCount) override;

        string getName() override {
            return "Degree3";
        }
    };

    class Degree3Distances {
    public:
        Degree3Distances(SteinerInstance* instance, const vector<node_id>& us, const unordered_set<node_id>& ignore) :
                instance_(instance), ignore_(ignore), q_(instance->getGraph()->getMaxKnownDistance()){
            for(auto n: us)
                q_.emplace(0, n, 0);
        }

        cost_id get(node_id target, cost_id limit);

    private:
        SteinerInstance* instance_;
        Queue<NodeWithCost> q_;
        const unordered_set<node_id>& ignore_;
        unordered_map<node_id, cost_id> dist_;
        cost_id cMax_ = 0;
    };
}


#endif //STEINER_DEGREE3REDUCTION_H
