//
// Created by aschidler on 1/22/20.
//

#ifndef DYNAMIC_STEINER_CPP_LABELSTORE_H
#define DYNAMIC_STEINER_CPP_LABELSTORE_H
#include <cstdint>
#include <boost/dynamic_bitset.hpp>
#include <list>
#include "LabelIterator.h"
#include "Steiner.h"

using namespace boost;

namespace steiner {
    class DynamicLabelStore {
    public:
        virtual ~DynamicLabelStore() {}
        DynamicLabelStore(node_id width, node_id nNodes) : width(width), nNodes(nNodes) {}
        virtual void addLabel(node_id node, const dynamic_bitset<> *newLabel) = 0;
        virtual DynamicLabelIterator* findLabels(node_id node, const dynamic_bitset<> *target) = 0;
    protected:
        unsigned int width;
        unsigned int nNodes;
    };
}
#endif //DYNAMIC_STEINER_CPP_LABELSTORE_H