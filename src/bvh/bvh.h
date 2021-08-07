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
#ifndef BVH_H_
#define BVH_H_

#include <algorithm>
#include <cstring>
#include <functional>
#include <vector>

#include "core/common/conf.h"

#define MEDIAN_PIVOT

class bvh;

// nearest structure, used in bvh closest query
struct nearest {
    point_t closest_point;
    primitive_t *closest_primitive;
    double sqr_distance;
    nearest() {
        sqr_distance = std::numeric_limits<double>::max();
    }
    void update_nearest_if_necessary(const point_t &new_point, primitive_t *new_primitive, double new_sqr_dis) {
        if (new_sqr_dis < sqr_distance) {
            this->closest_point = new_point;
            this->closest_primitive = new_primitive;
            this->sqr_distance = new_sqr_dis;
        }
    }
};

class bvh_travel_trait;

/// slow but easy to understand
class bvh // indeed, a node in bvh, abstraction of a convex shape.
{
public:
    typedef primitive_t *iterator;
    // hierarchy
    // future can use template for more general iterator
    bvh();

    // build bvh hierarchy tree on primitives in [beg, end)
    void build_bvh(primitive_t *beg, primitive_t *end);

    // initial bvh volume based on primitives in [beg, end)
    virtual void init_bounding_volume(const primitive_t *beg, const primitive_t *end) = 0;

    // create a node
    virtual std::shared_ptr<bvh> create_node() const = 0;

    // check whether a point is inside a bvh node
    // 0 no intersection, 1 intersect
    virtual bool intersect_to(const point_t &p) const = 0;

    // compute nearest information from bvh to given point
    // p: input query point
    // nearest: nearest information
    void compute_nearest_to(const point_t &p, nearest &n) const;

    void travel(bvh_travel_trait *trait) const;

    virtual ~bvh();

    // return the mid point of bvh
    point_t mid() const {
        return mid_;
    }

    bool is_leaf() const {
        return (primitive_ != nullptr) && (children_[0] == nullptr) && (children_[1] == nullptr);
    }

    std::shared_ptr<bvh> left_child() const {
        assert(children_[0] != nullptr);
        return children_[0];
    }
    std::shared_ptr<bvh> right_child() const {
        assert(children_[1] != nullptr);
        return children_[1];
    }
    primitive_t *get_primitive() const {
        assert(primitive_ != nullptr);
        return primitive_;
    }

    int size() const {
        return right_primitive_ - left_primitive_;
    }

    const iterator end() const {
        return right_primitive_;
    }

    const iterator begin() const {
        return left_primitive_ == nullptr ? end() : left_primitive_;
    }

    iterator end() {
        return right_primitive_;
    }

    iterator begin() {
        return left_primitive_ == nullptr ? end() : left_primitive_;
    }

    // calculate the squared exterior distance from point to bvh node, used in culling procedure
    virtual double squared_distance(const point_t &p) const = 0;

    // calculate the lower bound between primitive and bvh node
    virtual double squared_distance_lower_bound(const primitive_t &primitive) const = 0;

protected:
    // two type of tree split method
    // pivot: use geometry center to split, can caused unbalanced tree
    // sorter: use median center to split, the result is a balanced tree
    typedef std::function<bool(const primitive_t &)> pivot_t;
    virtual pivot_t *pivot(primitive_t *beg, primitive_t *end) const { return 0; }
    typedef std::function<bool(const primitive_t &, const primitive_t &)> sorter_t;
    virtual sorter_t *sorter(primitive_t *beg, primitive_t *end) const { return 0; }

    // if both children are null, it is leaf in this case, and there are
    // non-null primitives.
    std::shared_ptr<bvh> children_[2];
    primitive_t *primitive_;
    primitive_t *left_primitive_;
    primitive_t *right_primitive_;
    // geometry mid point of bvh node
    point_t mid_;
};

class aabb : public bvh {
public:
    virtual void init_bounding_volume(const primitive_t *beg, const primitive_t *end);
    virtual std::shared_ptr<bvh> create_node() const { return std::shared_ptr<bvh>(new aabb); }
    virtual bool intersect_to(const point_t &p) const;
    matrixd_t get_minmax() const {
        return minmax_;
    }
    virtual double squared_distance(const point_t &p) const;
    virtual double squared_distance_lower_bound(const primitive_t &primitive) const;

protected:
#ifdef MEDIAN_PIVOT
    virtual sorter_t *sorter(primitive_t *beg, primitive_t *end) const;
#else
    virtual pivot_t *pivot(primitive_t *beg, primitive_t *end) const;
#endif

    // for aabb tree, we use the vertices of diagonal to represent the bounding volume
    matrixd_t minmax_; // 3x2
};

// create root bvh node by type
bvh *create_bvh_node(const char *type);

class bvh_travel_trait {
public:
    // check whether current bvh node need travel futher
    // the return value is used to decide travel left or right child first
    // the lower one will travel first
    // if return value is less than 0, do not need travel
    virtual double calculate_travel_weight(const bvh *node) = 0;
    virtual bool need_travel(const bvh *node, const double weight) = 0;
    virtual void travel_leaf(const bvh *node) = 0;
    virtual ~bvh_travel_trait() {}
};

#endif
