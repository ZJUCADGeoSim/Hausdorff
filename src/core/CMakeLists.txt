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

file(GLOB zycgeo_core_source "${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp"
                             "${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.cpp"
                             "${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp")

add_library(zycgeo_core ${zycgeo_core_source})
target_link_libraries(zycgeo_core zjucad::matrix)