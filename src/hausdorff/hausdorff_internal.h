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

#ifndef HAUSDORFF_INTERNAL_H
#define HAUSDORFF_INTERNAL_H

#include "hausdorff.h"

double hausdorff(const bvh &node, const point_t &point);
void traverse(const bvh &A, const bvh &B, double &L, double &H, std::unique_ptr<hd_trait> &trait, point_t &max_point);

// implement voronoi subdivision
point_t voronoi_subdivide(const primitive_t &t1, const primitive_t &t2,
                          const point_t &e1, const point_t &e2,
                          const point_t &p1, const point_t &p2);

// this is the operator used in max priority queue
// we used the upper bound of approximated local hausdorff distance to sort the queue
// always deal with the one with maximum upper bound so as to reduce the global hausdorff bounds as fast as possible
bool operator<(const primitive_with_hd &left, const primitive_with_hd &right);

#endif