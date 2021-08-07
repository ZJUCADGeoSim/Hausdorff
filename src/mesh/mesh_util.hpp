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

#ifndef MESH_UTIL_HPP
#define MESH_UTIL_HPP

#include <vector>

#include "core/common/conf.h"
#include "mesh/mesh.hpp"

void build_primitive_array(const tri_mesh &mesh, std::vector<primitive_t> &tris) {
    const matrixd_t &v = *mesh.v_;
    const matrixst_t &t = *mesh.t_;
    tris.resize(t.size(2)); // resize to number of triangles
    for (size_t ti = 0; ti < tris.size(); ++ti) {
        tris[ti].points = v(zjucad::matrix::colon(), t(zjucad::matrix::colon(), ti)); // 3x3 matrix of coordinates of nodes
        const size_t n = tris[ti].points.size(2);                                     // can be used for more general setting
        tris[ti].barycenter = tris[ti].points * zjucad::matrix::ones<double>(n, 1) / n;
        tris[ti].id = ti;
        tris[ti].point_id.resize(3, 1);
        tris[ti].point_id = t(zjucad::matrix::colon(), ti);
    }
}

#endif