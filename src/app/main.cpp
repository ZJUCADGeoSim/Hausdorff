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

#include <chrono>
#include <functional>
#include <utility>

#include <io/io.h>

#include "bvh/bvh.h"
#include "core/geometry/bbox.hpp"
#include "hausdorff/hausdorff.h"
#include "hausdorff/stop_condition.h"
#include "mesh/adjacent_table.hpp"
#include "mesh/mesh_util.hpp"
#include <cxxopts.hpp>

using namespace std;
using namespace std::chrono;
using namespace zjucad::matrix;

LogLevel logs::_level = LogLevel::INFO;

enum class StopCondition {
    REL,
    ABS
};

int main(int argc, char **argv) // parse arguments and call Hausdorff
{

    cxxopts::Options options("Hausdorff Distance",
                             "Bounded Controlled Hausdorff Distance Calculator");
    options.add_options()("a,modelA", "Model A", cxxopts::value<std::string>())                                              //
        ("b,modelB", "Model B", cxxopts::value<std::string>())                                                               //
        ("e,error", "Error Tolerance, percentage of the diagronal of the bounding box of model A", cxxopts::value<double>()) //
        ("c,condition", "Stop conditon. Possible values: rel, diag, abs. Default: rel", cxxopts::value<std::string>())       //
        ("t,trait", "Calculation Trait", cxxopts::value<std::string>())                                                      //
        ("s,subdivision", "Subdivision Strategy (true to use voronoi subdivision)", cxxopts::value<bool>());

    auto result =
        options.parse(argc, argv); // TODO: exception error when input wrong
                                   // argument.  use boost::program_options
    if (result.count("help") || !result.count("modelA") ||
        !result.count("modelB")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    matrixd_t v[2];  // short for vertices
    matrixst_t t[2]; // short for triangles
    const string mesh_names[2] = {result["modelA"].as<std::string>(),
                                  result["modelB"].as<std::string>()};
    for (size_t m = 0; m < 2; ++m) {
        const bool status = !meshio::load_obj(mesh_names[m].c_str(), t[m], v[m]);
        logs(cout) << "[mesh_" << m << "_num_of_vertices] " << v[m].size(2)
                   << std::endl;
        logs(cout) << "[mesh_" << m << "_num_of_faces] " << t[m].size(2)
                   << std::endl;
        if (status == false || t[m].size(2) == 0 || t[m].size(1) != 3) {
            logs(cout) << "[Error] " << mesh_names[m]
                       << " cannot be parsed or is with 0 face." << std::endl;
            exit(-1);
        }
        logs(cout) << mesh_names[m] << " " << std::to_string(t[m].size(2))
                   << " triangles" << std::endl;
    }

    // build bvh hierarchy
    tri_mesh A = {&v[0], &t[0]}, B = {&v[1], &t[1]};

    StopCondition stop = StopCondition::REL;

    double error = 0;

    if (result.count("condition")) {
        if (result["condition"].as<std::string>() == "rel") {
            stop = StopCondition::REL;
            if (result.count("error")) {
                error = result["error"].as<double>();
            } else {
                error = 0.01;
            }
            logs(cout) << "[rel_error_bound] " << error << std::endl;
        } else if (result["condition"].as<std::string>() == "diag") {
            stop = StopCondition::ABS;
            double error_ratio = 0.01;
            if (result.count("error")) {
                error_ratio = result["error"].as<double>();
            }
            bbox box;
            box.add(*A.v_);
            error = sqrt(box.sqr_diagonal());
            logs(cout) << "[bbox_diagonal] " << error << std::endl;
            error *= error_ratio;
            if (error < 1e-12) {
                logs(cerr) << "expected error bound is too low, fix to 1e-12"
                           << std::endl;
                error = 1e-12;
            }
            logs(cout) << "[error_bound] " << error << std::endl;

        } else if (result["condition"].as<std::string>() == "abs") {
            stop = StopCondition::ABS;
            if (result.count("error")) {
                error = result["error"].as<double>();
            } else {
                error = 1e-12;
            }
            logs(cout) << "[error_bound] " << error << std::endl;
        } else {
            logs(cout) << "[Error] " << result["condition"].as<std::string>()
                       << " is not a valid stop condition." << std::endl;
            exit(-1);
        }
    } else {
        stop = StopCondition::REL;
        if (result.count("error")) {
            error = result["error"].as<double>();
        } else {
            error = 0.01;
        }
        logs(cout) << "[rel_error_bound] " << error << std::endl;
    }
    logs(cout) << "[stop_condition] " << (stop == StopCondition::ABS ? "abs" : "rel") << std::endl;

    // build bvh tree
    high_resolution_clock::time_point begin_clock = high_resolution_clock::now();
    tri_mesh meshes[2] = {A, B};
    vector<tri_with_id> tris[2];
    shared_ptr<bvh> pbvh[2];
    for (size_t m = 0; m < 2; ++m) {
        pbvh[m].reset(create_bvh_node("aabb"));
        build_primitive_array(meshes[m], tris[m]);
        pbvh[m]->build_bvh(&tris[m][0], &tris[m][0] + tris[m].size());
    }
    high_resolution_clock::time_point end_clock = high_resolution_clock::now();
    logs(cout)
        << INFO << "[bvh_build_cost] "
        << duration_cast<duration<double>>(end_clock - begin_clock).count() * 1000
        << std::endl;

    // build hd trait
    unique_ptr<hd_trait> trait;

    if (result.count("trait") &&
        result["trait"].as<std::string>().compare("triangle") == 0) {
        trait = unique_ptr<hd_trait>(
            new triangle_base_trait(A, tris[0], tris[1], pbvh[1]));
    } else {
        trait = unique_ptr<hd_trait>(
            new point_base_trait(A, tris[0], tris[1], pbvh[1]));
    }

    bool use_voronoi = false;
    if (result.count("subdivision")) {
        logs(cerr) << "use voronoi subdivision" << std::endl;
        use_voronoi = true;
    }

    function<bool(double, double)> stop_condition;
    if (stop == StopCondition::ABS) {
        stop_condition = [error](double L, double U) { return U - L < error; };
    } else {
        stop_condition = [error](double L, double U) { return U - L < L * error; };
    }

    hausdorff_result hd_result =
        hausdorff(A, B, pbvh, trait, use_voronoi, stop_condition);

    std::cout << std::endl
              << std::endl;
    logs(cout) << INFO << "########  calculation result  ########" << std::endl;
    {
        logs(cout) << "[first_travel_bound] " << hd_result.first_travel_bound.first
                   << " - " << hd_result.first_travel_bound.second << std::endl;
        logs(cout) << "[distance] " << hd_result.hausdorff_bound.first << " - "
                   << hd_result.hausdorff_bound.second << std::endl;
        logs(cout) << "[max_point] " << hd_result.max_point[0] << " "
                   << hd_result.max_point[1] << " " << hd_result.max_point[2]
                   << std::endl;
        logs(cout) << "[mean_distance] "
                   << (hd_result.hausdorff_bound.first +
                       hd_result.hausdorff_bound.second) /
                          2
                   << std::endl;
        logs(cout) << std::endl
                   << std::endl;
    }

    logs(cout) << "########  performance summary  ########" << std::endl;
    {
        logs(cout) << "[first_travel_cost] " << hd_result.first_travel_cost
                   << std::endl;
        logs(cout) << "[reduce_bound_cost] " << hd_result.bound_reduce_cost
                   << std::endl;
        logs(cout) << "[total_cost] "
                   << hd_result.first_travel_cost + hd_result.bound_reduce_cost
                   << std::endl;
        logs(cout) << "[voronoi_count] " << hd_result.voronoi_subdivision_count
                   << std::endl;
        logs(cout) << "[mid_subdivide_count] "
                   << hd_result.midpoint_subdivision_count << std::endl;
        logs(cout) << "[total_subdivision_count] "
                   << hd_result.midpoint_subdivision_count +
                          hd_result.voronoi_subdivision_count
                   << std::endl;
        logs(cout) << "[point_triangle_distance_count] " << point_triangle_count
                   << std::endl;
        logs(cout) << "[triangle_triangle_distance_count] "
                   << triangle_triangle_count << std::endl;
        logs(cout) << std::endl
                   << std::endl;
    }

    // reverse check
    logs(cout) << "########  reverse check  ########" << std::endl;
    {
        nearest_travel_trait trait(hd_result.max_point);
        pbvh[1]->travel(&trait);
        logs(cout) << "[reverse_check_result] " << sqrt(trait.result.sqr_distance)
                   << std::endl;
        logs(cout) << "[reverse_check_primitive_id] "
                   << trait.result.closest_primitive->id << std::endl;
    }
    return 0;
}
