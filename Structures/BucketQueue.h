//
// Created by aschidler on 2/5/20.
//

#ifndef STEINER_BUCKETQUEUE_H
#define STEINER_BUCKETQUEUE_H
#include "../Steiner.h"
#include "LinkedStack.h"

using namespace std;

namespace steiner {
    template<typename T>
    class BucketQueue {
    public :
        explicit BucketQueue(cost_id limit) : limit_(limit+1) {
            buckets_ = new LinkedStack<T>*[limit_]();
        }
        ~BucketQueue() {
            for(int i=0; i < limit_; i++)
                delete buckets_[i];

            delete[] buckets_;
        }

        template<typename... Ts>
        void emplace(cost_id cost, Ts&&... elem) {
            if (cost >= limit_)
                resize(cost);

            if (buckets_[cost] == nullptr) {
                buckets_[cost] = new LinkedStack<T>();
            }

            buckets_[cost]->emplace(std::forward<Ts>(elem)...);
            if (cost < pointer_)
                pointer_ = cost;
        }

        void push(cost_id cost, T& elem) {
            if (cost >= limit_)
                resize(cost);

            if (buckets_[cost] == nullptr) {
                buckets_[cost] = new LinkedStack<T>();
            }

            buckets_[cost]->push(elem);
            if (cost < pointer_)
                pointer_ = cost;
        }

        bool hasNext() {
            while(pointer_ < limit_ && (buckets_[pointer_] == nullptr || !buckets_[pointer_]->hasNext()))
                pointer_++;

            return pointer_ < limit_;
        }

        T dequeue() {
            auto elem = buckets_[pointer_]->pop();
            return elem;
        }

        T& peek() {
            hasNext();
            return buckets_[pointer_]->peek();
        }

    private:
        LinkedStack<T>** buckets_;
        cost_id limit_;
        cost_id pointer_ = 0;

        void resize(cost_id target) {
            auto newLimit = max(limit_ * 2, target);
            auto newarr = new LinkedStack<T>*[newLimit]();
            std::copy(buckets_, buckets_ + limit_, newarr);

            delete[] buckets_;
            limit_ = newLimit;
            buckets_ = newarr;
        }
    };
}


#endif //STEINER_BUCKETQUEUE_H
