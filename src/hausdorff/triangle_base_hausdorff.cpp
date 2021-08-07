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

// this file contains the implementation for triangle-based hausdorff calculation trait
// triangle-based hausdorff trait make culling on mesh B to retrieve the upper bound of
// local hausdorff distance

#include <typeinfo>
#include <vector>

#include <zjucad/matrix/matrix.h>

#include "core/common/conf.h"
#include "core/geometry/primitive_dis.hpp"
#include "hausdorff/face_base_travel_trait.hpp"
#include "hausdorff/hausdorff.h"
#include "hausdorff/hausdorff_internal.h"

using namespace zjucad::matrix;
using namespace std;

// utility function, used to deal with vector iterator and pure pointer
template <typename iterator>
const primitive_t *get_primitive(iterator iter) {
    return *iter;
}

template <>
const primitive_t *get_primitive(primitive_t *iter) {
    return iter;
}

template <>
const primitive_t *get_primitive(std::vector<primitive_t>::const_iterator iter) {
    return &*iter;
}

// check whether current bvh node need travelling
// need travelling principal:
// d(node_A, p) > global lower bound of HD
// p is the nearest point on B to node_A
double triangle_base_trait::need_travel(const bvh &A, const bvh &B, double &L, double &H) {
    // get the mid point of node A
    point_t mid = A.mid();

    // get the closest point on B to mid
    nearest n;
    B.compute_nearest_to(mid, n);

    // calculate the distance
    return hausdorff(A, n.closest_point);
}

void triangle_base_trait::shrink_bound(const primitive_t &primitive, const primitive_t &parent_primitive, const bvh &B, double &L, double &U, const double &parent_local_upper_sqr_bound, point_t &max_point) {
    // if L set for current primitive is not calculated, retrieve it by bvh travesal
    if (L_set.find(primitive.id) == L_set.end()) {
        face_base_travel_trait fb_travel_trait(&parent_primitive, parent_local_upper_sqr_bound);
        B.travel(&fb_travel_trait);
        L_set.emplace(primitive.id, fb_travel_trait.L);
    }
    std::unordered_map<size_t, vector<primitive_t *>>::iterator iter = L_set.find(primitive.id);

    point_t temp_vec = ones<double>(3, 1);
    temp_vec(0, 0) = temp_vec(1, 0) = temp_vec(2, 0) = std::numeric_limits<double>::max();

    double local_U = std::numeric_limits<double>::max(), local_L = 0;

    // iterate_leaf_inner(primitive, B, local_L, local_U, temp_vec);
    iterate_leaf_inner(primitive, iter->second.begin(), iter->second.end(), local_L, local_U, temp_vec);
    if (local_L > L) {
        L = local_L;
        size_t max_point_iter = max_element(temp_vec.begin(), temp_vec.end()) - temp_vec.begin();
        max_point = primitive.points(colon(), max_point_iter);
    }

    L = max(L, local_L);
    // maintain the priority heap
    if (local_U >= L) {
        primitive_with_hd pwhd(primitive, local_L, local_U);
        left_tris.push(pwhd);
    }
}

template <typename iterator>
void triangle_base_trait::iterate_leaf_inner(const primitive_t &primitive, iterator beg,
                                             iterator end, double &local_L, double &local_U, point_t &temp_vec) {
    // calculate the hausdorff distance with every primitive
    // calculate distance matrix
    size_t size = end - beg;
    matrixd_t dis_matrix = ones<double>(primitive.points.size(2), size);
    for (size_t point_iter = 0; point_iter != primitive.points.size(2); ++point_iter) {
        for (auto primitive_iter = beg; primitive_iter != end; ++primitive_iter) {
            dis_matrix(point_iter, primitive_iter - beg) = point_primitive_sqr_dis(primitive.points(colon(), point_iter), get_primitive(primitive_iter));
        }
    }

    // update upper bound
    for (size_t primitive_iter = 0; primitive_iter != dis_matrix.size(2); ++primitive_iter) {
        local_U = min(max(dis_matrix(colon(), primitive_iter)), local_U);
    }

    // update lower bound
    for (size_t point_iter = 0; point_iter != dis_matrix.size(1); ++point_iter) {
        temp_vec(point_iter, 0) = min(temp_vec(point_iter, 0), min(dis_matrix(point_iter, colon())));
    }
    local_L = max(temp_vec);
}

// iterate leaf node
// wrapper
void triangle_base_trait::iterate_leaf(const primitive_t &primitive, const bvh &B, double &L, double &U, point_t &max_point) {
    point_t temp_vec = ones<double>(3, 1);
    temp_vec(0, 0) = temp_vec(1, 0) = temp_vec(2, 0) = std::numeric_limits<double>::max();

    double local_U = std::numeric_limits<double>::max(), local_L = 0;

    iterate_leaf_inner(primitive, B, local_L, local_U, temp_vec);

    if (U == std::numeric_limits<double>::max()) {
        U = local_U;
    } else {
        U = max(U, local_U);
    }

    if (local_L > L) {
        L = local_L;
        size_t max_point_iter = max_element(temp_vec.begin(), temp_vec.end()) - temp_vec.begin();
        max_point = primitive.points(colon(), max_point_iter);
    }

    // maintain the priority heap
    if (local_U >= L) {
        primitive_with_hd pwhd(primitive, local_L, local_U);
        left_tris.push(pwhd);
    }
}

// iterate leaf node, real executor
// in triangle base trait, we need to make culling on bvh B
void triangle_base_trait::iterate_leaf_inner(const primitive_t &primitive, const bvh &B,
                                             double &local_L, double &local_U,
                                             point_t &temp_vec) {
    // how to calculate the distance between a primitive and a bvh node?
    // for now, I use the centroid of the primitive to calculate the distance between primitive and bvh node

    if (B.size() <= 4) {
        // calculate the hausdorff distance with every primitive in node B

        iterate_leaf_inner(primitive, B.begin(), B.end(), local_L, local_U, temp_vec);
    } else {
        // check whether we should go deeper
        // since we use mid point to represent the whole primitive, so the squared distance upper is lower than exact distance
        // PROBLEM: in ssv, this is not a promise, need further check!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        const double l_dis = B.left_child()->squared_distance_lower_bound(primitive);
        const double r_dis = B.right_child()->squared_distance_lower_bound(primitive);

        shared_ptr<bvh> children[2] = {B.left_child(), B.right_child()};
        const double *dis[2] = {&l_dis, &r_dis};

        // iterate the node with lower distance first
        if (l_dis > r_dis) {
            std::swap(children[0], children[1]);
            std::swap(dis[0], dis[1]);
        }
        for (size_t i = 0; i < 2; ++i) {
            if (*dis[i] <= local_U) {
                iterate_leaf_inner(primitive, *children[i], local_L, local_U, temp_vec);
            }
        }
    }
}