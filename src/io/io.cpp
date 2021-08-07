/*
	State Key Lab of CAD&CG Zhejiang Unv.

	Author: 
          Tengfei Jiang

	Copyright (c) 2004-2021 <Jin Huang>
	All rights reserved.

	Licensed under the MIT License.
*/

#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "io.h"

using namespace std;
using namespace zjucad::matrix;

namespace jtf {
namespace mesh {

static int analysis_obj(ifstream &obj_ifs, size_t &vertex_num,
                        size_t &face_num, size_t &face_type) // 3:triangle, 4: quad
{
    using namespace std;
    std::string strtemp;
    vertex_num = 0;
    face_num = 0;
    face_type = -1;

    vector<string> string_q;
    while (!obj_ifs.eof()) {
        std::getline(obj_ifs, strtemp);
        istringstream iss(strtemp.c_str());
        std::string cmd;
        iss >> cmd;
        if (cmd.length() == 0 || cmd[0] == '#')
            continue;
        if (cmd == "v" || cmd == "V") {
            if (strtemp.size() >= 2 && strtemp[1] == ' ')
                ++vertex_num;
        }
        if (cmd == "f" || cmd == "F") {
            ++face_num;
            // the following code has bug in wrong counting the spaces, reported by Yicun
            double v;
            size_t v_cnt = 0;
            for (; iss.good(); ++v_cnt)
                iss >> v;
            if (v_cnt != 3)
                cerr << v_cnt << endl;
            if (face_type == -1)
                face_type = v_cnt;
            else {
                if (face_type != v_cnt) {
                    cerr << "# [error] unsupported mesh: inconsistent face type" << endl;
                    return __LINE__;
                }
            }
        }
        strtemp.clear();
    }

    obj_ifs.clear();
    obj_ifs.seekg(0, std::ios::beg);

    if (face_type == 3)
        cerr << "# [info] triangle mesh " << endl;
    else if (face_type == 4)
        cerr << "# [info] quad mesh " << endl;
    cerr << "# [info] vertex " << vertex_num << " face " << face_num << endl;

    return 0;
}

int load_obj(const char *filename, matrixst &faces, matrixd &nodes) {
    ifstream obj_ifs(filename);
    if (obj_ifs.fail()) {
        std::cerr << "# open " << filename << " fail. " << std::endl;
        return __LINE__;
    }

    std::string strtemp;
    size_t pointsnum = 0;
    size_t polygonsnum = 0;
    size_t face_type = -1;

    if (analysis_obj(obj_ifs, pointsnum, polygonsnum, face_type))
        return __LINE__;

    if (face_type != 3 && face_type != 4) {
        cerr << "# error in jtfmesh: only support triangle/quad mesh << " << face_type << endl;
        return __LINE__;
    }

    nodes.resize(3, pointsnum);
    faces.resize(face_type, polygonsnum);
    size_t idxpoints = 0;
    size_t idxpolygons = 0;

    while (!obj_ifs.eof()) {
        obj_ifs >> strtemp;
        if (obj_ifs.eof())
            break;
        if (strtemp.empty()) {
            break;
        }
        if (strtemp == "#" || strtemp[0] == '#') {
            std::getline(obj_ifs, strtemp);
            strtemp.clear();
            continue;
        }
        if (strtemp == "v" || strtemp == "V") {
            for (size_t i = 0; i < 3; ++i) {
                obj_ifs >> nodes(i, idxpoints);
            }
            ++idxpoints;
        }
        if (strtemp == "f" || strtemp == "F") {
            std::getline(obj_ifs, strtemp);
            std::stringstream sss(strtemp);
            std::string temp_str;
            size_t vidx = 0;
            while (sss >> temp_str) {
                if (vidx == face_type) {
                    std::cerr << "# not trimesh" << std::endl;
                    return __LINE__;
                }
                int num = count(temp_str.begin(), temp_str.end(), '/');
                if (num == 0) {
                    faces(vidx, idxpolygons) = atoi(temp_str.c_str()) - 1;
                } else if (num == 1 || num == 2) {
                    temp_str.erase(temp_str.begin() + temp_str.find('/', 0), temp_str.end());
                    faces(vidx, idxpolygons) = atoi(temp_str.c_str()) - 1;
                    //faces[idxpolygons].push_back( atoi(temp_str.c_str())-1);
                }
                ++vidx;
            }
            ++idxpolygons;
        }
        strtemp.clear();
    }
    obj_ifs.close(); // finish reading obj
    return 0;
}

} // namespace mesh
} // namespace jtf
