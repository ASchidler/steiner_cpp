//
// Created by aschidler on 1/23/20.
//

#ifndef STEINER_LABELITERATOR_H
#define STEINER_LABELITERATOR_H

#include "Steiner.h"

using namespace boost;

namespace steiner {
    class LabelIterator {
    public:
        virtual ~LabelIterator() {}
        virtual const dynamic_bitset<> &operator*() = 0;
        virtual dynamic_bitset<> *operator->() = 0;
        virtual LabelIterator &operator++() = 0;
        virtual bool hasNext() = 0;
        //virtual const ConstLabelIterator operator++(int) = 0;
    };
}

#endif //STEINER_LABELITERATOR_H
