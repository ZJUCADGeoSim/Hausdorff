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
#include <iostream>
#include <memory>
#include <numeric>

#include <zjucad/matrix/io.h>

#include "core/common/n_matrix.h"
#include "core/geometry/bbox.hpp"
#include "core/geometry/primitive_dis.hpp"

#include "bvh.h"

using namespace std;
using namespace zjucad::matrix;

bvh::bvh() {
    children_[0] = 0;
    children_[1] = 0;
    primitive_ = 0;
}

bvh::~bvh() {
}

void bvh::build_bvh(primitive_t *beg, primitive_t *end) {
    if (!beg || !end || beg == end) { // invalid or null input
        cerr << beg << " " << end << endl;
        assert(0); // it is a logical error instead of a runtime error.
        exit(-1);
    }

    this->init_bounding_volume(beg, end); // call specific instance

    left_primitive_ = beg;
    right_primitive_ = end;
    if (end - beg == 1) {
        primitive_ = beg;
        return;
    }

    // partition into children_
    //this->partition(beg, end);
    unique_ptr<pivot_t> p(pivot(beg, end));
    unique_ptr<sorter_t> s(sorter(beg, end));
    primitive_t *m = 0;
    if (!!p) {
        m = std::partition(beg, end, *p);
        if (m - beg == 0 || end - m == 0) {
            cerr << "WARNING: single child due to unbalanced split." << endl;
            return;
        }
    } else if (!!s) {
        m = beg + (end - beg) / 2;
        nth_element(beg, m, end, *s);
    }

    this->children_[0] = create_node();
    this->children_[0]->build_bvh(beg, m);
    this->children_[1] = create_node();
    this->children_[1]->build_bvh(m, end);
}

void bvh::travel(bvh_travel_trait *trait) const {
    // leaf node of non leaf node check
    assert(primitive_ != 0 || (children_[0] != 0 && children_[1] != 0));

    // leaf node
    if (primitive_ != 0) {
        trait->travel_leaf(this);
        return;
    }

    double dis_to[2] = {std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max()};
    for (size_t i = 0; i < 2; ++i) {
        dis_to[i] = trait->calculate_travel_weight(children_[i].get());
    }

    const size_t m = min_element(dis_to, dis_to + 2) - dis_to;
    for (size_t i = 0; i < 2; ++i) {
        const size_t side = (m + i) % 2;
        if (trait->need_travel(children_[side].get(), dis_to[side])) {
            children_[side]->travel(trait);
        }
    }
}

void bvh::compute_nearest_to(const point_t &p, nearest &n) const {
    // only one point
    assert(p.size(1) == 3 && p.size(2) == 1);
    // leaf node of non leaf node check
    assert(primitive_ != 0 || (children_[0] != 0 && children_[1] != 0));

    // leaf node
    if (primitive_ != 0) {
        point_t cp;
        double dis = point_primitive_sqr_dis(p, this->primitive_, cp);
        n.update_nearest_if_necessary(cp, this->primitive_, dis);
        return;
    }

    // non leaf node
    bool looked[2] = {false, false};
    double dis_to[2] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};

    for (size_t i = 0; i < 2; ++i) {
        // travel if poin is inside aabb box
        if (this->children_[i]->intersect_to(p)) {

            this->children_[i]->compute_nearest_to(p, n);

            looked[i] = true;
        } else {
            dis_to[i] = this->children_[i]->squared_distance(p);
        }
    }

    // pick the one closer to children node
    const size_t m = min_element(dis_to, dis_to + 2) - dis_to;
    for (size_t i = 0; i < 2; ++i) {
        const size_t side = (m + i) % 2;
        // travel if node is not culled
        if (looked[side] == false && dis_to[side] < n.sqr_distance) {
            this->children_[side]->compute_nearest_to(p, n);
        }
    }
}

bvh *create_bvh_node(const char *type) {
    unique_ptr<bvh> p;
    if (!strcmp(type, "aabb")) {
        p.reset(new aabb);
    }
    return p.release();
}
