//
// Created by aschidler on 1/22/20.
//

#ifndef DYNAMIC_STEINER_HASHSETLABELSTORE_H
#define DYNAMIC_STEINER_HASHSETLABELSTORE_H

#include "Steiner.h"
#include "LabelStore.h"
#include <bits/stdc++.h>

using namespace boost;
using namespace std;
using namespace steiner;


namespace steiner {
    class DynamicHashSetLabelIterator : public DynamicLabelIterator {
    public:
        explicit DynamicHashSetLabelIterator(unordered_set<dynamic_bitset<>>::iterator start,
                unordered_set<dynamic_bitset<>>::iterator end, const dynamic_bitset<>* target) :
                pos(start), end(end), target(target) {
            findNext();
        }
        const dynamic_bitset<>& operator*() override;
        dynamic_bitset<>* operator->() override;
        DynamicLabelIterator& operator++() override;
        bool hasNext() override;
    private:
        unordered_set<dynamic_bitset<>>::iterator pos;
        unordered_set<dynamic_bitset<>>::iterator end;
        const dynamic_bitset<>* target;
        void findNext();
    };

    class DynamicHashSetLabelStore : public DynamicLabelStore {
    public:
        DynamicHashSetLabelStore(node_id width, node_id nNodes) : DynamicLabelStore(width, nNodes) {
            this->labels_ = new unordered_set<dynamic_bitset<>>[nNodes];

            for (size_t i = 0; i < nNodes; i++) {
                this->labels_[i] = unordered_set<dynamic_bitset<>>();
            }
        }

        ~DynamicHashSetLabelStore() override {
            delete[] this->labels_;
        }

        void addLabel(node_id node, const dynamic_bitset<> *newLabel) override;

        DynamicHashSetLabelIterator* findLabels(node_id node, const dynamic_bitset<> *target) override;

    //private:
        unordered_set<dynamic_bitset<>> *labels_;
    };
}

#endif //DYNAMIC_STEINER_HASHSETLABELSTORE_H
