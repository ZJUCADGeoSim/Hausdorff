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

#include <fstream>

#include "core/geometry/bbox.hpp"

#include "core/geometry/primitive_dis.hpp"
#include "hausdorff_internal.h"

using namespace std;
using namespace zjucad::matrix;

// subdivide given edge by voronoi split
point_t voronoi_subdivide(const primitive_t &t1, const primitive_t &t2,
                          const point_t &e1, const point_t &e2,
                          const point_t &p1, const point_t &p2) {
    point_t edge = e1 - e2;
    point_t normal1 = cross((t1.points(colon(), 2) - t1.points(colon(), 0)), (t1.points(colon(), 1) - t1.points(colon(), 0)));
    point_t normal2 = cross((t2.points(colon(), 2) - t2.points(colon(), 0)), (t2.points(colon(), 1) - t2.points(colon(), 0)));

    if (dot(normal1, normal2) < 0) {
        normal2 = -normal2;
    }

    normal1 /= norm(normal1);
    normal2 /= norm(normal2);
    point_t angle_median = normal1 + normal2;
    point_t median_normal = cross(angle_median, edge);
    double d = dot(median_normal, e1);
    double t = (d - dot(median_normal, p1)) / (dot(median_normal, (p2 - p1)));

    if (t >= 0.95 || t <= 0.05) {
        t = 0.5;
    }

    return (p2 - p1) * t + p1;
}

// this is the operator used in max priority queue
// we used the upper bound of approximated local hausdorff distance to sort the queue
// always deal with the one with maximum upper bound so as to reduce the global hausdorff bounds as fast as possible
bool operator<(const primitive_with_hd &left, const primitive_with_hd &right) {
    // max(primitive.U)
    return left.U < right.U;
}
