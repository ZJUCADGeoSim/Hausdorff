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
#include <zjucad/matrix/io.h>

#include "core/common/n_matrix.h"
#include "core/geometry/bbox.hpp"
#include "core/geometry/primitive_dis.hpp"

#include "bvh.h"

using namespace std;
using namespace zjucad::matrix;

double aabb::squared_distance(const point_t &p) const {
    double dis = 0, aux = 0;

    for (size_t d = 0; d < p.size(1); ++d) {
        // less than min point
        if (p(d, 0) < minmax_(d, 0)) {
            aux = minmax_(d, 0) - p(d, 0);
            dis += aux * aux;
        }
        // more than max point
        else if (p(d, 0) > minmax_(d, 1)) {
            aux = p(d, 0) - minmax_(d, 1);
            dis += aux * aux;
        }
    }
    return dis;
}

double aabb::squared_distance_lower_bound(const primitive_t &primitive) const {
    bbox box;
    box.add(primitive.points);
    double sqr_dis = 0;

    for (size_t d_iter = 0; d_iter < 3; ++d_iter) {
        if (box.minmax_(d_iter, 0) > minmax_(d_iter, 1)) {
            sqr_dis += (box.minmax_(d_iter, 0) - minmax_(d_iter, 1)) * (box.minmax_(d_iter, 0) - minmax_(d_iter, 1));
        } else if (box.minmax_(d_iter, 1) < minmax_(d_iter, 0)) {
            sqr_dis += (minmax_(d_iter, 0) - box.minmax_(d_iter, 1)) * (minmax_(d_iter, 0) - box.minmax_(d_iter, 1));
        }
    }
    return sqr_dis;
}

bool aabb::intersect_to(const point_t &p) const {
    assert(minmax_.size(1) == p.size(1));
    for (size_t di = 0; di < minmax_.size(1); ++di)
        if (p[di] < minmax_(di, 0) || p[di] > minmax_(di, 1))
            return false;
    return true;
}

void aabb::init_bounding_volume(const primitive_t *beg, const primitive_t *end) {
    minmax_.resize(3, 2);
    minmax_(colon(), 0) = std::numeric_limits<double>::max();
    minmax_(colon(), 1) = -std::numeric_limits<double>::max();

    // iterate all vertices
    for (auto iter = beg; iter < end; ++iter) {
        for (size_t pi = 0; pi < iter->points.size(2); ++pi) {
            for (size_t di = 0; di < iter->points.size(1); ++di) {
                if (minmax_(di, 0) > iter->points(di, pi))
                    minmax_(di, 0) = iter->points(di, pi);
                if (minmax_(di, 1) < iter->points(di, pi))
                    minmax_(di, 1) = iter->points(di, pi);
            }
        }
    }

    // assign mid
    mid_ = (minmax_(colon(), 0) + minmax_(colon(), 1)) / 2;
}

#ifdef MEDIAN_PIVOT
bvh::sorter_t *aabb::sorter(primitive_t *beg, primitive_t *end) const { // does this realy binds to aabb?  Except for minmax_, it looks also generally applied.
    const matrixd_t range = minmax_(colon(), 1) - minmax_(colon(), 0);
    const size_t d = max_element(range.begin(), range.end()) - range.begin();
    unique_ptr<bvh::sorter_t> s(new bvh::sorter_t);
    *s = [d](const primitive_t &a, const primitive_t &b) {
        return a.barycenter(d, 0) < b.barycenter(d, 0);
    };
    return s.release();
}
#else
bvh::pivot_t *aabb::pivot(primitive_t *beg, primitive_t *end) const { // does this realy binds to aabb?  Except for minmax_, it looks also generally applied.
    matrixd_t range = minmax_(colon(), 1) - minmax_(colon(), 0);
    size_t d = max_element(range.begin(), range.end()) - range.begin();
    double c = (minmax_(d, 1) + minmax_(d, 0)) / 2;
    unique_ptr<bvh::pivot_t> p(new bvh::pivot_t);
    *p = [d, c](const primitive_t &p) {
        return p.barycenter[d] < c;
    };
    return p.release();
}
#endif