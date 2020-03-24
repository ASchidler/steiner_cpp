//
// Created by andre on 22.03.20.
//

#ifndef STEINER_GRAPHPRUNER_H
#define STEINER_GRAPHPRUNER_H

#include "../Steiner.h"
#include "../SteinerInstance.h"
#include "VoronoiDiagram.h"
#include "ShortestPath.h"
#include "../Reductions/Reducer.h"

using namespace steiner;
namespace steiner {
    class GraphPruner {
    public:
        GraphPruner(SteinerInstance& instance) : instance_(instance),
            reducer_(Reducer::getMinimalReducer(&instance, false))
        {
        };

        bool prune();
        void reduce();
        std::shared_ptr<SteinerResult> approximate();
        SteinerInstance& getInstance() {
            return instance_;
        }
    private:
        SteinerInstance& instance_;
        Reducer reducer_;
        std::shared_ptr<SteinerResult> lastResult_ = std::shared_ptr<SteinerResult>(nullptr);
    };
}


#endif //STEINER_GRAPHPRUNER_H
