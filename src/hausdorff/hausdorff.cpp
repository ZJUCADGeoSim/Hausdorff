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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>

#include "zjucad/matrix/io.h"

#include "bvh/nearest_query.h"
#include "core/geometry/primitive_dis.hpp"
#include "hausdorff.h"
#include "hausdorff/closest_cache.hpp"
#include "hausdorff_internal.h"
#include "mesh/adjacent_table.hpp"

using namespace std;
using namespace zjucad::matrix;
using namespace std::chrono;

void traverse(const bvh &A, const bvh &B, double &L, double &H, unique_ptr<hd_trait> &trait, point_t &max_point) {
    // leaf node
    if (A.is_leaf()) {
        trait->iterate_leaf(*A.get_primitive(), B, L, H, max_point);
        return;
    }

    // non leaf node
    if (!A.is_leaf()) { // TODO: why this if is necessary?  You have check this in previous lines

        // TODO: you used array (children_array, but not use array for dis.  It is better to make them consistent
        const double l_dis = trait->need_travel(*A.left_child(), B, L, H);
        const double r_dis = trait->need_travel(*A.right_child(), B, L, H);

        shared_ptr<bvh> children_array[2] = {A.left_child(), A.right_child()};
        const double *dis_array[2] = {&l_dis, &r_dis};

        // the node with larger distance has more possibility to improve the lower bound of hausdorff distance higher
        // so we travel the one with higher distance
        if (l_dis < r_dis) {
            std::swap(children_array[0], children_array[1]);
            std::swap(dis_array[0], dis_array[1]);
        }
        for (size_t i = 0; i < 2; ++i) {
            if (*dis_array[i] > L) {
                traverse(*children_array[i], B, L, H, trait, max_point);
            }
        }
    }
}

void subdivide(tri_mesh &A,
               closest_cache &cache,
               primitive_adjacent_table &adj_table,
               shared_ptr<bvh> B,
               const tri_mesh &model_B,
               const primitive_t &prim,
               bool use_voronoi,
               primitive_t result[4],
               size_t &voronoi_count,
               size_t &mid_count) {

    point_t new_vertex[3];
    size_t new_vertex_id[3];
    for (size_t i = 0; i < 3; ++i) {
        new_vertex_id[i] = A.is_subdivision_vertex_exist(prim.point_id[i], prim.point_id[(i + 1) % 3]);
        if (new_vertex_id[i] != -1) {
            new_vertex[i] = A.get_vertex(new_vertex_id[i]);
        } else {
            bool voronoi_valid = false;

            if (use_voronoi) {
                primitive_t *t[2];
                for (size_t point_iter = 0; point_iter < 2; ++point_iter) {
                    size_t point_id = (i + point_iter) % 3;
                    t[point_iter] = cache.get(prim.point_id[point_id], prim.points(colon(), point_id));
                }
                auto edge_itr = adj_table.table_.find(primitive_pair(t[0]->id, t[1]->id));
                if (edge_itr != adj_table.table_.cend()) {
                    voronoi_count++;
                    new_vertex[i] = voronoi_subdivide(
                        *t[0], *t[1],
                        (*model_B.v_)(colon(), edge_itr->second.first), (*model_B.v_)(colon(), edge_itr->second.second),
                        prim.points(colon(), i), prim.points(colon(), (i + 1) % 3));
                    voronoi_valid = true;
                }
            }
            if (!voronoi_valid) {
                mid_count++;
                new_vertex[i] = (prim.points(colon(), i) + prim.points(colon(), (i + 1) % 3)) / 2;
            }
            new_vertex_id[i] = A.add_subdivision_vertex(prim.point_id[i], prim.point_id[(i + 1) % 3], new_vertex[i]);
            // update closest cache
            cache.get(new_vertex_id[i], new_vertex[i]);
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        result[i].points(colon(), 1) = new_vertex[i];
        result[i].points(colon(), 0) = prim.points(colon(), i);
        result[i].points(colon(), 2) = new_vertex[(i + 2) % 3];
        result[i].point_id[0] = prim.point_id[i];
        result[i].point_id[1] = new_vertex_id[i];
        result[i].point_id[2] = new_vertex_id[(i + 2) % 3];
        result[i].id = prim.id;
    }

    for (size_t d = 0; d < 3; ++d) {
        result[3].points(colon(), d) = new_vertex[d];
        result[3].point_id(d, 0) = new_vertex_id[d];
        result[3].id = prim.id;
    }
}

hausdorff_result hausdorff(tri_mesh &A, const tri_mesh &B,
                           shared_ptr<bvh> pbvh[2], unique_ptr<hd_trait> &trait,
                           bool use_voronoi, function<bool(double, double)> &stop_condition) {
    // build primitive adjacent table
    primitive_adjacent_table adjacent_table;
    build_primitive_adjacent_table_from_mesh(*B.v_, *B.t_, adjacent_table);

    // first iteration
    high_resolution_clock::time_point begin_clock = high_resolution_clock::now();
    double L = 0, U = std::numeric_limits<double>::max();

    point_t max_point = ones<double>(3, 1);
    traverse(*pbvh[0], *pbvh[1], L, U, trait, max_point);

    // TODO: design a class for more convinient time measure: void
    // clock.start(), double clock.stop() returns the milisecond since
    // the last call of start.
    high_resolution_clock::time_point end_clock = high_resolution_clock::now();

    hausdorff_result result;
    // update result structure
    result.first_travel_bound = make_pair(sqrt(L), sqrt(U));
    result.first_travel_cost = duration_cast<duration<double>>(end_clock - begin_clock).count() * 1000;

    // subdivide to shrink error bound
    begin_clock = high_resolution_clock::now();

    size_t voronoi_count = 0, mid_count = 0;
    // vector<double> new_edge; vector<size_t> edge_index;

    // while( ((!use_relative_error && (sqrt(U)-sqrt(L) > error)) || (use_relative_error && ((sqrt(U)-sqrt(L)) >= (sqrt(U)+sqrt(L))/2 * 0.01)))
    logs(cout) << "[culling rate] " << 1 - (trait->left_tris.size() * 1.0 / A.t_->size(2)) << std::endl;
    while (!stop_condition(sqrt(L), sqrt(U)) && (!trait->left_tris.empty())) {
        // std::cout << "count: " << voronoi_count + mid_count << std::endl;
        primitive_with_hd pwhd = trait->left_tris.top();
        trait->left_tris.pop();
        U = trait->left_tris.top().U;

        // skip primitive whose upper bound is lower than global lower bound
        if (pwhd.U < L) {
            // max point already found, exit
            U = L;
            break;
        }

        // subdivide triangle into four, and update model
        primitive_t t[4];

        subdivide(A, trait->closest_cache_, adjacent_table, pbvh[1], B, pwhd.prim, use_voronoi, t, voronoi_count, mid_count);

        // calculate local bound
        // #pragma omp parallel for
        for (size_t i = 0; i < 4; ++i) {
            if ((trait->closest_cache_.get(t[i].point_id(0, 0)) == trait->closest_cache_.get(t[i].point_id(1, 0))) &&
                (trait->closest_cache_.get(t[i].point_id(1, 0)) == trait->closest_cache_.get(t[i].point_id(2, 0)))) {
                continue;
            }
            // std::cout << "travel child: " << i << std::endl;
            trait->shrink_bound(t[i], pwhd.prim, *pbvh[1], L, U, pwhd.U, max_point);
        }
        U = trait->left_tris.top().U;
    }

    end_clock = high_resolution_clock::now();

    result.max_point = max_point;
    result.bound_reduce_cost = duration_cast<duration<double>>(end_clock - begin_clock).count() * 1000;
    result.hausdorff_bound = make_pair(sqrt(L), sqrt(U));
    result.voronoi_subdivision_count = voronoi_count;
    result.midpoint_subdivision_count = mid_count;
    // {
    //   ofstream debug_line("debug_line.vtk");
    //   line2vtk(debug_line, new_edge.data(), new_edge.size()/3, edge_index.data(), edge_index.size()/2);
    //   debug_line.close();
    // }

    return result;
}

// similar to closest_point
// calculate hausdorff distance from bvh node to point
// used in traversal check
double hausdorff(const bvh &node, const point_t &p) {
    {
        const aabb *pt = dynamic_cast<const aabb *>(&node);
        if (!!pt) {
            // aabb node
            point_t mid_point = pt->mid();
            point_t max_vector = p - mid_point;
            point_t mid_corner_vector = mid_point - pt->get_minmax()(colon(), 0);
            for (size_t d = 0; d < p.size(1); ++d) {
                max_vector(d, 0) = fabs(max_vector(d, 0)) + fabs(mid_corner_vector(d, 0));
            }
            return dot(max_vector, max_vector); //max_vector(0, 0) * max_vector(0, 0) + max_vector(1, 0) * max_vector(1, 0) + max_vector(2, 0) * max_vector(2, 0);
        }
    }

    logs(cout) << ERROR
               << "unknown bvh type" << std::endl;
    exit(-1);
}
