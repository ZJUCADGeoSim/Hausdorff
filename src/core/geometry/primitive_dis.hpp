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

#ifndef PRIMITIVE_DIS_H
#define PRIMITIVE_DIS_H

#include <iostream>

#include "core/common/conf.h"

// calculate sqr distance from point primitive
// I try to rewrite this function, different from former implement,
// use the method introduce in Real-time collision detection, Ericson, Chapter 5,
// also refer from libigl
double point_primitive_sqr_dis(const point_t &point, const primitive_t *prim, point_t &closest_point);
inline double point_primitive_sqr_dis(const point_t &point, const primitive_t *prim) {
    point_t useless;
    return point_primitive_sqr_dis(point, prim, useless);
}

double segment_segment_sqr_dis(const point_t &p1, const point_t &q1, const point_t &p2, const point_t &q2, point_t &c1, point_t &c2);
inline double segment_segment_sqr_dis(const point_t &p1, const point_t &q1, const point_t &p2, const point_t &q2) {
    point_t useless1, useless2;
    return segment_segment_sqr_dis(p1, q1, p2, q2, useless1, useless2);
}

double primitive_primitive_sqr_dis(const primitive_t &prim1, const primitive_t &prim2);

double point_segment_sqr_dis(const point_t &p, const point_t &s0, const point_t &s1);

#endif
