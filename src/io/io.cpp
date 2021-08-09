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
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "io.h"

#include "igl/readOBJ.h"

using namespace std;
using namespace zjucad::matrix;

namespace meshio {

int load_obj(const char *filename, matrixst &faces, matrixd &nodes) {
    Eigen::MatrixXd V, F;
    if (!igl::readOBJ(filename, V, F)) {
        return -1;
    }

    nodes.resize(3, V.rows());
    faces.resize(F.cols(), F.rows());

    for (size_t i = 0; i < V.rows(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            nodes(j, i) = V(i, j);
        }
    }
    for (size_t i = 0; i < F.rows(); ++i) {
        for (size_t j = 0; j < F.cols(); ++j) {
            faces(j, i) = F(i, j);
        }
    }
    cerr << "# [info] vertex " << V.rows() << " face " << F.rows() << endl;

    return 0;
}

} // namespace meshio
