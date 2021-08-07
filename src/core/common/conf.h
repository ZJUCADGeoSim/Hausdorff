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

#ifndef _BDH_CONF_H_
#define _BDH_CONF_H_

// TODO: one can also use Eigen.  I am just more familar with this
// matrix lib.

#include <limits>
#include <memory>
#include <utility>

#include <zjucad/matrix/io.h>
#include <zjucad/matrix/matrix.h>

#include "log_helper.h"
#include "n_matrix.h"

typedef zjucad::matrix::matrix<double> matrixd_t;
typedef zjucad::matrix::matrix<size_t> matrixst_t;
typedef zjucad::matrix::n_matrix<double, 3> point_t;
typedef zjucad::matrix::n_matrix<double, 12> rectangle_t;
typedef std::pair<size_t, size_t> edge_t;
// typedef matrixd_t point_t;
// typedef matrixd_t primitive_t; // simplex: point, line, tri, tet

struct tri_with_id {
    zjucad::matrix::n_matrix<double, 9> points;
    zjucad::matrix::n_matrix<size_t, 3> point_id;
    point_t barycenter;
    size_t id;
    tri_with_id() {
        points = zjucad::matrix::zeros<double>(3, 3);
        point_id = zjucad::matrix::zeros<double>(3, 1);
        barycenter = zjucad::matrix::zeros<double>(3, 1);
        id = -1;
    }
};
typedef tri_with_id primitive_t;

// performance record
extern long point_triangle_count;
extern long triangle_triangle_count;

#endif
