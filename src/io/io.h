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

#ifndef JTF_MESH_IO_H
#define JTF_MESH_IO_H

#include <deque>
#include <fstream>
#include <iostream>
#include <zjucad/matrix/matrix.h>

namespace meshio {

typedef zjucad::matrix::matrix<double> matrixd;
typedef zjucad::matrix::matrix<size_t> matrixst;

///
/// @brief load_obj, it only support pure triangle/quad mesh
/// @param filename
/// @param mesh output mesh matrix
/// @param node output node matrix
/// @return return 0 if fine or non-zeros
///
int load_obj(const char *filename, matrixst &mesh, matrixd &node);

} // namespace meshio

#endif //TRIMESH_IO_H
