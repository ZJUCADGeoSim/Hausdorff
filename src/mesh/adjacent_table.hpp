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

#ifndef ADJACENT_TABLE_HPP_
#define ADJACENT_TABLE_HPP_

#include "core/common/conf.h"
#include "mesh/definition.hpp"

// store adjacent primitives information
struct primitive_adjacent_table {
    // key: primitive pair, value: edge
    std::unordered_map<primitive_pair, edge_t, primitive_hasher> table_;
};

bool get_neighbor_edge(const matrixst_t &prims, size_t p1, size_t p2, edge_t &e);

void build_primitive_adjacent_table_from_mesh(const matrixd_t &v, const matrixst_t &prims, primitive_adjacent_table &table);

#endif