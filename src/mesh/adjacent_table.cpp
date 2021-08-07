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

#include "adjacent_table.hpp"

using namespace zjucad::matrix;

bool get_neighbor_edge(const matrixst_t &prims, size_t p1, size_t p2, edge_t &e) {
    size_t point_count = 0;
    for (size_t vi = 0; vi < prims.size(1); ++vi) {
        for (size_t vj = 0; vj < prims.size(1); ++vj) {
            if (prims(vi, p1) == prims(vj, p2)) {
                if (point_count == 0) {
                    e.first = prims(vi, p1);
                } else if (point_count == 1) {
                    e.second = prims(vi, p1);
                } else {
                    return false;
                }
                point_count++;
            }
        }
    }
    if (point_count != 2) {
        return false;
    } else {
        return true;
    }
}

void build_primitive_adjacent_table_from_mesh(const matrixd_t &v, const matrixst_t &prims, primitive_adjacent_table &table) {
    // build vertex-primitives map
    std::vector<std::vector<size_t>> vpmap(v.size(2));
    for (size_t pi = 0; pi < prims.size(2); ++pi) {
        for (size_t vi = 0; vi < prims.size(1); ++vi) {
            vpmap[prims(vi, pi)].push_back(pi);
        }
    }

    // iterate vertex to build primitive adjacent table
    for (size_t i = 0; i < vpmap.size(); ++i) {
        if (vpmap[i].size() <= 1)
            continue;
        for (size_t p1 = 0; p1 < vpmap[i].size(); ++p1) {
            for (size_t p2 = p1 + 1; p2 < vpmap[i].size(); ++p2) {
                edge_t e;
                if (get_neighbor_edge(prims, vpmap[i][p1], vpmap[i][p2], e)) {
                    // std::cout << "prim " << prims(colon(), vpmap[i][p1]) << prims(colon(), vpmap[i][p2]) << std::endl;
                    // std::cout << "e: " << e.first << " " << e.second << std::endl;
                    table.table_.emplace(primitive_pair(vpmap[i][p1], vpmap[i][p2]), e);
                }
            }
        }
    }

    // for(auto i = table.table_.begin(); i != table.table_.end(); ++i){
    //     std::cout << i->first.pair_.first << "-" << i->first.pair_.second << ": " << i->second.first << ", " << i->second.second << std::endl;
    // }
    // std::cout << "size: " << table.table_.size() << std::endl;
}