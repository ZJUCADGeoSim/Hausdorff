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

#ifndef _NEAREST_QUERY_H
#define _NEAREST_QUERY_H

#include "bvh/bvh.h"

class nearest_travel_trait : public bvh_travel_trait {
public:
    virtual double calculate_travel_weight(const bvh *node);
    virtual bool need_travel(const bvh *node, const double weight);
    virtual void travel_leaf(const bvh *node);

    nearest_travel_trait(const point_t &query_point)
        : query_point(query_point) {}
    virtual ~nearest_travel_trait() {}

    nearest result;

private:
    const point_t query_point;
};

#endif