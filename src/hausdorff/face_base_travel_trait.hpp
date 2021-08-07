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

#ifndef _FACE_BASE_TRAVEL_TRAIT_
#define _FACE_BASE_TRAVEL_TRAIT_

#include "bvh/bvh.h"

class face_base_travel_trait : public bvh_travel_trait {
public:
    face_base_travel_trait(const primitive_t *primitive, const double local_upper_sqr_hausdorff_distance) : primitive_(primitive),
                                                                                                            lu_sqr_h_dis(local_upper_sqr_hausdorff_distance) {
    }

    virtual ~face_base_travel_trait() {
    }

    virtual double calculate_travel_weight(const bvh *node);
    virtual bool need_travel(const bvh *node, const double weight);
    virtual void travel_leaf(const bvh *node);

    std::vector<primitive_t *> L;

private:
    const primitive_t *primitive_;
    const double lu_sqr_h_dis;
};

#endif // _FACE_BASE_TRAVEL_TRAIT_