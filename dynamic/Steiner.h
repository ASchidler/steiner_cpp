//
// Created by andre on 24.01.20.
//

#ifndef DYNAMIC_STEINER_STEINER_H
#define DYNAMIC_STEINER_STEINER_H
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS

#include "../Steiner.h"
#include <boost/dynamic_bitset.hpp>
using namespace boost;

namespace std {
    /**
     * Allows the use of dynamic bitsets in hashed structures (set, map)
     */
    template <typename Block, typename Alloc> struct hash<boost::dynamic_bitset<Block, Alloc> > {
        size_t operator()(boost::dynamic_bitset<Block, Alloc> const& bs) const {
            return boost::hash_value(bs.m_bits);
        }
    };
};

#endif //DYNAMIC_STEINER_STEINER_H
