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

// TODO: Add typedefs for node identifier, costs, etc. to quickly change from int64 to 32 etc.
#endif //STEINER_STEINER_H
