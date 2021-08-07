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

#include "core/geometry/bbox.hpp"
#include "core/geometry/primitive_dis.hpp"

#include "face_base_travel_trait.hpp"
#include "hausdorff_internal.h"

double face_base_travel_trait::calculate_travel_weight(const bvh *node) {
    return node->squared_distance_lower_bound(*primitive_);
}

bool face_base_travel_trait::need_travel(const bvh *node, const double weight) {
    return weight < lu_sqr_h_dis;
}

void face_base_travel_trait::travel_leaf(const bvh *node) {
    double dis = primitive_primitive_sqr_dis(*primitive_, *node->get_primitive());
    if (dis < lu_sqr_h_dis) {
        L.push_back(node->get_primitive());
    }
}