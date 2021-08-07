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

#ifndef HAUSDORFF_H
#define HAUSDORFF_H

#define USE_TRAIT

#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "bvh/bvh.h"
#include "bvh/nearest_query.h"
#include "core/util/timer.hpp"
#include "hausdorff/closest_cache.hpp"
#include "hausdorff/stop_condition.h"
#include "mesh/adjacent_table.hpp"
#include "mesh/mesh.hpp"

// external interfaces to the whole program
void subdivide(tri_mesh &A,
               closest_cache &cache,
               primitive_adjacent_table &adj_table,
               std::shared_ptr<bvh> B,
               const tri_mesh &model_B,
               const primitive_t &prim,
               bool use_voronoi,
               primitive_t result[4],
               size_t &voronoi_count,
               size_t &mid_count);

// hausdorff calculation trait, use different method to calculate hausdorff distance
class hd_trait;

class hausdorff_result {
public:
    std::pair<double, double> hausdorff_bound;
    std::pair<double, double> first_travel_bound;
    point_t max_point;
    double first_travel_cost;
    double bound_reduce_cost;
    size_t voronoi_subdivision_count;
    size_t midpoint_subdivision_count;
};

// calculate hausdorff distance from mesh A to mesh B
// input:
//   A: mesh A
//   B: mesh B
//   error: squared error tolerance between upper and lower bounds
//   hd_trait: the method used in hausdorff distance calculation
hausdorff_result hausdorff(tri_mesh &A,
                           const tri_mesh &B,
                           std::shared_ptr<bvh> pbvh[2],
                           std::unique_ptr<hd_trait> &hd_trait,
                           bool use_voronoi, std::function<bool(double, double)> &stop_condition);

// structure to store local hausdorff distance bound for single primitive
struct primitive_with_hd {
    primitive_t prim;
    double L;
    double U;
    primitive_with_hd(primitive_t prim, const double L, const double U)
        : prim(prim), L(L), U(U) {}
    primitive_with_hd() {}
};

class hd_trait {
public:
    hd_trait(const tri_mesh &A, const std::vector<primitive_t> &tri_A,
             const std::vector<primitive_t> &tri_B, std::shared_ptr<bvh> ref_bvh) : closest_cache_(ref_bvh, A.v_->size(2)), A_(A), tri_A_(tri_A), tri_B_(tri_B) {
    }

    virtual double need_travel(const bvh &A, const bvh &B, double &L, double &H) = 0;
    virtual void iterate_leaf(const primitive_t &primitive, const bvh &B, double &L, double &H, point_t &max_point) = 0;
    virtual void shrink_bound(const primitive_t &primitive, const primitive_t &parent_primitive, const bvh &B, double &L, double &H, const double &parent_local_upper_sqr_bound, point_t &max_point) = 0;

    virtual ~hd_trait() {}
    // left triangles, used to record survived triangles after culling
    std::priority_queue<primitive_with_hd> left_tris;
    // closest cache, used to reduce duplicate closest point search

    // std::vector<primitive_t*> closest_cache;
    closest_cache closest_cache_;
    // mesh A, used to get vertices id, cooperate with closest cache
    const tri_mesh &A_;
    const std::vector<primitive_t> &tri_A_;
    const std::vector<primitive_t> &tri_B_;
    accumulate_timer hd_timer;
};

// point based hausdorff calculation trait, our algorithm
class triangle_base_trait : public hd_trait {
public:
    triangle_base_trait(const tri_mesh &A, const std::vector<primitive_t> &tri_A,
                        const std::vector<primitive_t> &tri_B, std::shared_ptr<bvh> ref_bvh) : hd_trait(A, tri_A, tri_B, ref_bvh) {}
    virtual double need_travel(const bvh &A, const bvh &B, double &L, double &H);
    virtual void iterate_leaf(const primitive_t &primitive, const bvh &B, double &L, double &H, point_t &max_point);
    virtual void shrink_bound(const primitive_t &primitive, const primitive_t &parent_primitive, const bvh &B, double &L, double &H, const double &parent_local_upper_sqr_bound, point_t &max_point);
    virtual ~triangle_base_trait() {}

private:
    void iterate_leaf_inner(const primitive_t &primitive, const bvh &B,
                            double &local_L, double &local_H,
                            point_t &temp_vec);

    template <typename iterator>
    void iterate_leaf_inner(const primitive_t &primitive, iterator beg,
                            iterator end, double &local_L, double &local_H, point_t &temp_vec);

    // L set, the meaning is the same to min tang's paper, used to accelerate the subdivision procedure
    std::unordered_map<size_t, std::vector<primitive_t *>> L_set;
};

// face based hausdorff calculation trait, implementent Min Tang's algorithm
class point_base_trait : public hd_trait {
public:
    point_base_trait(const tri_mesh &A, const std::vector<primitive_t> &tri_A,
                     const std::vector<primitive_t> &tri_B, std::shared_ptr<bvh> ref_bvh) : hd_trait(A, tri_A, tri_B, ref_bvh) {}

    virtual double need_travel(const bvh &A, const bvh &B, double &L, double &H);
    virtual void iterate_leaf(const primitive_t &primitive, const bvh &B, double &L, double &H, point_t &max_point);
    virtual void shrink_bound(const primitive_t &primitive, const primitive_t &parent_primitive, const bvh &B, double &L, double &H, const double &parent_local_upper_sqr_bound, point_t &max_point);

    virtual ~point_base_trait() {}
};

#endif
