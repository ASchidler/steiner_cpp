//
// Created by aschidler on 2/5/20.
//

#ifndef STEINER_BUCKETQUEUE_H
#define STEINER_BUCKETQUEUE_H
#include "../Steiner.h"

using namespace std;

namespace steiner {
    template<class T>
    class BucketQueue {
        BucketQueue(cost_id limit) {
            buckets_ = new vector<T>[limit];
        }
        ~BucketQueue() {
            delete[] buckets_;
        }

        void enqueue(cost_id cost, T& elem) {
            move(elem, back_inserter(buckets_[cost]));
            if (cost < pointer_)
                pointer_ = cost;
        }

        bool hasNext() {
            while(buckets_[pointer_].empty() && pointer_ < limit_)
                pointer_++;

            return pointer_ < limit_;
        }

        T dequeue() {
            while(buckets_[pointer_].empty())
                pointer_++;

            auto elem = buckets_[pointer_].back();
            buckets_[pointer_].pop_back();

            return elem;
        }
    private:
        vector<T>* buckets_;
        cost_id limit_;
        cost_id pointer_ = 0;
    };
}


#endif //STEINER_BUCKETQUEUE_H
