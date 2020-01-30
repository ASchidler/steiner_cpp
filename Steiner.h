//
// Created by andre on 24.01.20.
//

#ifndef STEINER_STEINER_H
#define STEINER_STEINER_H
#include <cstdint>
#include <boost/functional/hash.hpp>
#include <boost/dynamic_bitset.hpp>
#include <bits/stdc++.h>

using namespace boost;

typedef uint16_t node_id;
typedef uint32_t cost_id; // TODO: 32 or 64 bit?
//#define MAXCOST (__UINT64_C(18446744073709551615))
#define MAXCOST 4294967295U
namespace std {
    /**
     * Allows the use of dynamic bitsets in hashed structures (set, map)
     */
    template <typename Block, typename Alloc> struct hash<boost::dynamic_bitset<Block, Alloc> > {
        size_t operator()(boost::dynamic_bitset<Block, Alloc> const& bs) const {
            size_t seed = boost::hash_value(bs.size());

            std::vector<Block> blocks(bs.num_blocks());
            boost::hash_range(seed, blocks.begin(), blocks.end());

            return seed;
        }
    };

    /**
     * Allows the use of const dynamic bitsets in hashed structures (set, map)
     */
    template <typename Block, typename Alloc> struct hash<const boost::dynamic_bitset<Block, Alloc> > {
        size_t operator()(const boost::dynamic_bitset<Block, Alloc> & bs) const {
            size_t seed = boost::hash_value(bs.size());

            std::vector<Block> blocks(bs.num_blocks());
            boost::hash_range(seed, blocks.begin(), blocks.end());

            return seed;
        }
    };
};

namespace steiner {
    struct DoubleCostEntry {
        DoubleCostEntry(node_id node, cost_id totalCost, cost_id edgeCost) : node(node), totalCost(totalCost), edgeCost(edgeCost) {
        }

        node_id node;
        cost_id totalCost;
        cost_id edgeCost;

        // TODO: These are actually the wrong way so that priority queues are min queues...
        bool operator<(const DoubleCostEntry& p2) const {
            return totalCost > p2.totalCost ||
                   // Choosing shorter edges over longer ones often provides better results
                   (totalCost == p2.totalCost && edgeCost < p2.edgeCost) ||
                   // This is just a tie breaker
                   (totalCost == p2.totalCost && edgeCost == p2.edgeCost && node < p2.node);
        }
    };
    struct DoubleNodeEntry {
        DoubleNodeEntry(node_id n1, node_id n2, cost_id cost) :n1(n1), n2(n2), cost(cost) {
        }

        node_id n1;
        node_id n2;
        cost_id cost;

        // TODO: These are actually the wrong way so that priority queues are min queues...
        bool operator<(const DoubleNodeEntry& p2) const {
            return cost > p2.cost ||
                   // Choosing shorter edges over longer ones often provides better results
                   (cost == p2.cost && n1 < p2.n1) ||
                   // This is just a tie breaker
                   (cost == p2.cost && n1 == p2.n2 && n1 < p2.n2);
        }
    };
}
#endif //STEINER_STEINER_H
