/*
	State Key Lab of CAD&CG Zhejiang Unv.

	Author: 
          Yicun Zheng (3130104113@zju.edu.cn)
          Haoran Sun (hrsun@zju.edu.cn)
          Jin Huang (hj@cad.zju.edu.cn)

	Copyright (c) 2004-2021 <Jin Huang>
	All rights reserved.

	Licensed under the MIT License.
*/

#ifndef _DEFINITION_HPP
#define _DEFINITION_HPP

#include <unordered_map>
#include <utility>

#include "core/common/conf.h"

struct primitive_pair {
    std::pair<size_t, size_t> pair_;
    primitive_pair(size_t t1, size_t t2) {
        if (t1 < t2)
            pair_ = std::make_pair(t1, t2);
        else
            pair_ = std::make_pair(t2, t1);
    }
    bool operator==(const primitive_pair &other) const {
        return (this->pair_.first == other.pair_.first) && (this->pair_.second == other.pair_.second);
    }
};

struct primitive_hasher {
    std::size_t operator()(const primitive_pair &primitive) const {
        return (std::hash<size_t>()(primitive.pair_.first) ^
                (std::hash<size_t>()(primitive.pair_.second) << 1));
    }
};

struct edge_hasher {
    std::size_t operator()(const edge_t &edge) const {
        return (std::hash<size_t>()(edge.first)) ^
               (std::hash<size_t>()(edge.second) << 1);
    }
};

#endif