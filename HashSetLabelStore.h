//
// Created on 1/22/20.
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
    template <typename T>
    class HashSetLabelIterator : public LabelIterator<T> {
    public:
        explicit HashSetLabelIterator(typename unordered_set<T>::iterator start,
                typename unordered_set<T>::iterator end, const T target) :
                pos(start), end(end), target(target) {
            findNext();
        }
        const T operator*() override;
        const T operator->() override;
        LabelIterator<T>& operator++() override;
        bool hasNext() override;
    private:
        typename unordered_set<T>::iterator pos;
        typename unordered_set<T>::iterator end;
        const T target;
        void findNext();
    };

    template <typename T>
    class HashSetLabelStore : public LabelStore<T> {
    public:
        HashSetLabelStore(node_id width, node_id nNodes) : LabelStore<T>(width, nNodes) {
            this->labels_ = new unordered_set<T>[nNodes];

            for (size_t i = 0; i < nNodes; i++) {
                this->labels_[i] = unordered_set<T>();
            }
        }

        ~HashSetLabelStore() override {
            delete[] this->labels_;
        }

        void addLabel(node_id node, const T newLabel) override;

        HashSetLabelIterator<T>* findLabels(node_id node, const T target) override;

    //private:
        unordered_set<T> *labels_;
    };
}

#endif //STEINER_HASHSETLABELSTORE_H
