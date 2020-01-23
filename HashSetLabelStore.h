//
// Created by aschidler on 1/22/20.
//

#ifndef STEINER_HASHSETLABELSTORE_H
#define STEINER_HASHSETLABELSTORE_H

#include "LabelStore.h"
#include <boost/dynamic_bitset.hpp>
#include <boost/functional/hash.hpp>
#include <bits/stdc++.h>

using namespace boost;
using namespace std;
using namespace steiner;

namespace std {

    template <typename Block, typename Alloc> struct hash<boost::dynamic_bitset<Block, Alloc> > {
        size_t operator()(boost::dynamic_bitset<Block, Alloc> const& bs) const {
            size_t seed = boost::hash_value(bs.size());

            std::vector<Block> blocks(bs.num_blocks());
            boost::hash_range(seed, blocks.begin(), blocks.end());

            return seed;
        }
    };

}

namespace steiner {
    class HashSetLabelIterator : public LabelIterator {
    public:
        explicit HashSetLabelIterator(unordered_set<dynamic_bitset<>>::iterator start,
                unordered_set<dynamic_bitset<>>::iterator end, dynamic_bitset<>* target) :
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
        dynamic_bitset<>* target;
        void findNext();
    };

    class HashSetLabelStore : public LabelStore {
    public:
        HashSetLabelStore(unsigned int width, unsigned int nNodes) : LabelStore(width, nNodes) {
            this->labels_ = new unordered_set<dynamic_bitset<>>[nNodes];

            for (size_t i = 0; i < nNodes; i++) {
                this->labels_[i] = unordered_set<dynamic_bitset<>>();
            }
        }

        ~HashSetLabelStore() {
            delete[] this->labels_;
        }

        void addLabel(unsigned int node, dynamic_bitset<> *newLabel) override;

        HashSetLabelIterator* findLabels(unsigned int node, dynamic_bitset<> *target) override;

    private:
        unordered_set<dynamic_bitset<>> *labels_;
    };
}

#endif //STEINER_HASHSETLABELSTORE_H
