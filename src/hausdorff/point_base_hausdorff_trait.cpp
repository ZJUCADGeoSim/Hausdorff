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

// this file contains the implementation for point-based hausdorff calculation trait
// point-based hausdorff trait uses closest primitives on mesh B to retrieve the upper bound of
// local hausdorff distance

#include <iomanip>
#include <vector>

#include <zjucad/matrix/matrix.h>

#include "core/common/conf.h"
#include "core/geometry/primitive_dis.hpp"
#include "hausdorff/hausdorff.h"
#include "hausdorff/hausdorff_internal.h"

using namespace zjucad::matrix;
using namespace std;

double point_base_trait::need_travel(const bvh &A, const bvh &B, double &L, double &H) {
    point_t mid_point = A.mid();

#ifdef USE_TRAIT
    nearest_travel_trait nearest_trait(mid_point);
    B.travel(&nearest_trait);
    return hausdorff(A, nearest_trait.result.closest_point);
#else
    nearest n;
    B.compute_nearest_to(mid_point, n);
    return hausdorff(A, n.closest_point);
#endif
}

void point_base_trait::shrink_bound(const primitive_t &primitive, const primitive_t &parent_primitive, const bvh &B, double &L, double &U, const double &parent_local_upper_sqr_bound, point_t &max_point) {
    iterate_leaf(primitive, B, L, U, max_point);
}

// IDEA: is it possible to use current bounds to reduce some distance computation?
void point_base_trait::iterate_leaf(const primitive_t &primitive, const bvh &B, double &L, double &U, point_t &max_point) {
    // find closest point on B to each vertices of primitive
    vector<primitive_t *> closest_primitives;

    closest_primitives.resize(primitive.points.size(2));

    for (size_t i = 0; i < primitive.points.size(2); ++i)
        closest_primitives[i] = (closest_cache_.get(primitive.point_id[i], primitive.points(colon(), i)));

    {
        point_t mid_point = primitive.points * ones<double>(3, 1) / 3;

#ifdef USE_TRAIT
        nearest_travel_trait nearest_trait(mid_point);
        B.travel(&nearest_trait);
        closest_primitives.push_back(nearest_trait.result.closest_primitive);
#else
        nearest n;
        B.compute_nearest_to(mid_point, n);
        closest_primitives.push_back(n.closest_primitive);
#endif
    }

    // calculate new upper and lower bound
    matrixd_t dis_matrix(closest_primitives.size(), primitive.points.size(2));

    // #pragma omp parallel for
    for (size_t tri_on_B = 0; tri_on_B < closest_primitives.size(); ++tri_on_B) {
        for (size_t point_on_A = 0; point_on_A < primitive.points.size(2); ++point_on_A) {
            dis_matrix(tri_on_B, point_on_A) = point_primitive_sqr_dis(primitive.points(colon(), point_on_A), closest_primitives[tri_on_B]);
        }
    }
    // local lower bound
    double local_lower_bound = std::numeric_limits<double>::min();
    size_t max_point_iter = -1;
    for (size_t i = 0; i < min(dis_matrix.size(1), dis_matrix.size(2)); ++i) {
        if (local_lower_bound < dis_matrix(i, i)) {
            local_lower_bound = dis_matrix(i, i);
            max_point_iter = i;
        }
    }

    // local upper bound
    double local_upper_bound = std::numeric_limits<double>::max();
    for (size_t i = 0; i < dis_matrix.size(1); ++i) {
        local_upper_bound = min(local_upper_bound, max(dis_matrix(i, colon())));
    }

    // update bound if necessary
    if (local_lower_bound > L) {
        L = local_lower_bound;
        max_point = primitive.points(colon(), max_point_iter);
    }

    if (U == std::numeric_limits<double>::max()) {
        U = local_upper_bound;
    } else {
        U = max(local_upper_bound, U);
    }

    // update left primitives
    if (local_upper_bound >= L) {
        primitive_with_hd pwhd(primitive, local_lower_bound, local_upper_bound);
        left_tris.push(pwhd);
    }
}