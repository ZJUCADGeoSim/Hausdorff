#	State Key Lab of CAD&CG Zhejiang Unv.
#
#	Author: 
#          Yicun Zheng (3130104113@zju.edu.cn)
#          Haoran Sun (hrsun@zju.edu.cn)
#          Jin Huang (hj@cad.zju.edu.cn)
#
#	Copyright (c) 2004-2021 <Jin Huang>
#	All rights reserved.
#
#	Licensed under the MIT License.

file(GLOB meshio_src "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(meshio ${meshio_src})
target_link_libraries(meshio zjucad::matrix igl::core Eigen3::Eigen)
