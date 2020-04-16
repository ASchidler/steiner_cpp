//
// Created by aschidler on 2/5/20.
//

#ifndef STEINER_BUCKETQUEUE_H
#define STEINER_BUCKETQUEUE_H
#include "../Steiner.h"

using namespace std;

namespace steiner {
    template<typename T>
    class BucketQueue {
    public :
        explicit BucketQueue(cost_id limit) {
            buckets_ = new vector<T>*[limit]();
        }
        ~BucketQueue() {
            for(int i=0; i < limit_; i++)
                delete buckets_[i];

            delete[] buckets_;
        }

        template<typename... Ts>
        void enqueue(cost_id cost, Ts&&... elem) {
            if (buckets_[cost] == nullptr) {
                buckets_[cost] = new vector<T>();
            }

            buckets_[cost]->emplace_back(std::forward<Ts>(elem)...);
            if (cost < pointer_)
                pointer_ = cost;
        }

        bool hasNext() {
            while((buckets_[pointer_] == nullptr || buckets_[pointer_]->empty()) && pointer_ < limit_)
                pointer_++;

            return pointer_ < limit_;
        }

        T dequeue() {
            auto elem = buckets_[pointer_]->back();
            buckets_[pointer_]->pop_back();

            return elem;
        }
    private:
        vector<T>** buckets_;
        cost_id limit_{};
        cost_id pointer_ = 0;
    };
}


#endif //STEINER_BUCKETQUEUE_H
