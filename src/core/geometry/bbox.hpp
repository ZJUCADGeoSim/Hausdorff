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

#ifndef _BBOX_HPP
#define _BBOX_HPP

#include "core/common/conf.h"

// bounding box
struct bbox {

    bbox() {
        minmax_.resize(3, 2);
        minmax_(zjucad::matrix::colon(), 0) = std::numeric_limits<double>::max();
        minmax_(zjucad::matrix::colon(), 1) = -std::numeric_limits<double>::max();
    }

    // column based, every column is a point
    void add(const matrixd_t &points) {
        assert(points.size(1) == 3);
        for (size_t d_iter = 0; d_iter < 3; ++d_iter) {
            minmax_(d_iter, 0) = std::min(min(points(d_iter, zjucad::matrix::colon())), minmax_(d_iter, 0));
            minmax_(d_iter, 1) = std::max(max(points(d_iter, zjucad::matrix::colon())), minmax_(d_iter, 1));
        }
    }

    double diagonal() const {
        return sqrt(sqr_diagonal());
    }

    double sqr_diagonal() const {
        point_t diagonal = minmax_(zjucad::matrix::colon(), 1) - minmax_(zjucad::matrix::colon(), 0);
        return dot(diagonal, diagonal);
    }

    matrixd_t minmax_;
};

#endif