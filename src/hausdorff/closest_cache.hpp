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

#ifndef _CLOSEST_CACHE_HPP
#define _CLOSEST_CACHE_HPP

#include <iostream>
#include <memory>
#include <vector>

#include "bvh/bvh.h"
#include "bvh/nearest_query.h"
#include "core/common/log_helper.h"

// TODO: the interface does not like typical cache.  Try to use the
//following interface: add(point_t), get(id).  Handling the cache miss
//in get(id) instead of halding it in caller.  So closest_cache may
//need to hold refernce to pwhd.prim.points.
class closest_cache {
public:
    closest_cache(std::shared_ptr<bvh> ref_bvh, size_t point_size) : ref_bvh_(ref_bvh) {
        closest_cache_ = std::vector<primitive_t *>(point_size * 2, nullptr);
    }

    primitive_t *get(size_t id) {
        return closest_cache_[id];
    }

    primitive_t *get(size_t id, const point_t &point) {
        if (id >= closest_cache_.size()) {
            closest_cache_.resize(id * 2, nullptr);
        }

        if (closest_cache_[id] == nullptr) {
            nearest_travel_trait query(point);
            ref_bvh_->travel(&query);
            closest_cache_[id] = query.result.closest_primitive;
        }
        return closest_cache_[id];
    }

    size_t size() {
        return closest_cache_.size();
    }

private:
    std::vector<primitive_t *> closest_cache_;
    std::shared_ptr<bvh> ref_bvh_;
};

#endif
