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

file(GLOB zycgeo_hausdorff_src "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(zycgeo_hausdorff ${zycgeo_hausdorff_src})
target_link_libraries(zycgeo_hausdorff zycgeo_core 
                                       zycgeo_mesh 
                                       zycgeo_bvh)
