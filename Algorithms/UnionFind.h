//
// Created by aschidler on 1/30/20.
//

#ifndef STEINER_UNIONFIND_H
#define STEINER_UNIONFIND_H

#include "../Steiner.h"

namespace steiner {
    class UnionFind {
    public:
        UnionFind(node_id limit) : limit_(limit){
            elements_ = new UFElement_[limit];
            for(node_id i=0; i < limit; i++) {
                elements_[i].value = i;
                elements_[i].parent_ = i;
            }
        }
        ~UnionFind() {
            delete [] elements_;
        }

        node_id find(node_id value) {
            return find_(value)->value;
        }

        bool unionize(node_id value1, node_id value2) {
            auto* el1 = find_(value1);
            auto* el2 = find_(value2);

            if (el1->value == el2->value)
                return false;

            if (el1->rank < el2->rank) {
                std::swap(el1, el2);
            }
            el2->parent_ = el1->value;

            if (el1->rank == el2->rank)
                el1->rank++;

            return true;
        }
    private:
        struct UFElement_ {
            UFElement_() {

            }
            node_id value = 0;
            node_id rank = 0;
            node_id parent_ = 0;
        };
        node_id limit_;
        UFElement_* elements_;

        UFElement_* find_(node_id value) {
            auto* el = &elements_[value];
            while(el->parent_ != el->value)
                el = &elements_[el->parent_];

            return el;
        }

    };
}
#endif //STEINER_UNIONFIND_H
