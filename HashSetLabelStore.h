//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_HASHSETLABELSTORE_H
#define STEINER_HASHSETLABELSTORE_H

#include "Steiner.h"
#include "LabelStore.h"
#include <bits/stdc++.h>

using namespace boost;
using namespace std;
using namespace steiner;


namespace steiner {
    class HashSetLabelIterator : public LabelIterator {
    public:
        explicit HashSetLabelIterator(unordered_set<dynamic_bitset<>>::iterator start,
                unordered_set<dynamic_bitset<>>::iterator end, const dynamic_bitset<>* target) :
                pos(start), end(end), target(target) {
            findNext();
        }
        const dynamic_bitset<>& operator*() override;
        dynamic_bitset<>* operator->() override;
        LabelIterator& operator++() override;
        bool hasNext() override;
    private:
        unordered_set<dynamic_bitset<>>::iterator pos;
        unordered_set<dynamic_bitset<>>::iterator end;
        const dynamic_bitset<>* target;
        void findNext();
    };

    class HashSetLabelStore : public LabelStore {
    public:
        HashSetLabelStore(node_id width, node_id nNodes) : LabelStore(width, nNodes) {
            this->labels_ = new unordered_set<dynamic_bitset<>>[nNodes];

            for (size_t i = 0; i < nNodes; i++) {
                this->labels_[i] = unordered_set<dynamic_bitset<>>();
            }
        }

        ~HashSetLabelStore() override {
            delete[] this->labels_;
        }

        void addLabel(node_id node, const dynamic_bitset<> *newLabel) override;

        HashSetLabelIterator* findLabels(node_id node, const dynamic_bitset<> *target) override;

    //private:
        unordered_set<dynamic_bitset<>> *labels_;
    };
}

#endif //STEINER_HASHSETLABELSTORE_H
