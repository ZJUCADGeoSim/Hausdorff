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

#ifndef MESH_CONF_HPP
#define MESH_CONF_HPP

#include <iostream>
#include <stdexcept>
#include <unordered_map>

#include "core/common/conf.h"
#include "mesh/definition.hpp"

class tri_mesh {
public:
    tri_mesh(matrixd_t *v, matrixst_t *t) : v_(v), t_(t) {}

    // add subdivision vertex
    // parent_1 one parent vertex id
    // parent_2 the other parent vertex id
    // point the new vertex coordinate
    // return: the new vertex id
    size_t add_subdivision_vertex(size_t parent_1, size_t parent_2, point_t &point) {
        if (parent_1 > parent_2) {
            std::swap(parent_1, parent_2);
        }
        // check whether exist
        int exist_id = is_subdivision_vertex_exist(parent_1, parent_2);
        if (exist_id == -1) {
            extra_v_.push_back(point);
            size_t new_id = extra_v_.size() + (*v_).size(2) - 1;
            parent_child_map_.emplace(edge_t(parent_1, parent_2), new_id);
            return new_id;
        } else {
            return exist_id;
        }
    }

    // get vertex coordinate by id
    point_t get_vertex(size_t id) {
        if (id >= (v_->size(2) + extra_v_.size())) {
            std::cerr << "id out of bound: " << id << ", limit: " << v_->size(2) + extra_v_.size() << std::endl;
            exit(-1);
        }

        if (id < v_->size(2)) {
            return (*v_)(zjucad::matrix::colon(), id);
        }

        if (id >= v_->size(2)) {
            return extra_v_[id - v_->size(2)];
        }
        throw std::logic_error("get_vertex: id is not in the range of v_");
    }

    // check whether exist a subdivided vertex between parent_1 and parent_2
    int is_subdivision_vertex_exist(size_t parent_1, size_t parent_2) {
        if (parent_1 > parent_2) {
            std::swap(parent_1, parent_2);
        }
        edge_t edge(parent_1, parent_2);
        auto iter = parent_child_map_.find(edge);
        if (iter != parent_child_map_.cend()) {
            return iter->second;
        } else {
            return -1;
        }
    }

    // inheritance compatibility
    matrixd_t *v_;
    matrixst_t *t_;

private:
    std::vector<point_t> extra_v_;
    std::unordered_map<edge_t, int, edge_hasher> parent_child_map_;
};

#endif