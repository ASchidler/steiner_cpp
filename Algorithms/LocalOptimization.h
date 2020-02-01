//
// Created by andre on 31.01.20.
//

#ifndef STEINER_LOCALOPTIMIZATION_H
#define STEINER_LOCALOPTIMIZATION_H
#include "../Graph.h"

namespace steiner {
    class LocalOptimization {
    public:
        void vertexInsertion(Graph* dg, Graph* tr);
        void pathExchange();
        void keyVertexDeletion();
    };
}


#endif //STEINER_LOCALOPTIMIZATION_H
