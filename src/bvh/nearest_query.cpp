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

#include "bvh/nearest_query.h"
#include "core/geometry/primitive_dis.hpp"

double nearest_travel_trait::calculate_travel_weight(const bvh *node) {
    if (node->intersect_to(query_point)) {
        return 0;
    } else {
        return node->squared_distance(query_point);
    }
}

bool nearest_travel_trait::need_travel(const bvh *node, const double weight) {
    // if point is inside node, need travel, 0 means highest travel priority
    return weight < result.sqr_distance;
}

void nearest_travel_trait::travel_leaf(const bvh *node) {
    point_t cp;
    double sqr_dis = point_primitive_sqr_dis(query_point, node->get_primitive(), cp);
    result.update_nearest_if_necessary(cp, node->get_primitive(), sqr_dis);
}